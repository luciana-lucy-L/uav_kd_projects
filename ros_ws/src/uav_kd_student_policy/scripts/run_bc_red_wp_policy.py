#!/home/l/miniconda3/envs/dbc241/bin/python3
"""
run_bc_red_wp_policy.py — BC-RedWP student policy deployment node

Subscribes:
  /camera/color/image_raw               (sensor_msgs/Image)
  /dynamicNavigation/bspline_trajectory (nav_msgs/Path, world frame)
  /CERLAB/quadcopter/pose               (geometry_msgs/PoseStamped)

Publishes:
  /CERLAB/quadcopter/cmd_vel  (geometry_msgs/TwistStamped)
  /CERLAB/quadcopter/takeoff  (std_msgs/Empty)
  /CERLAB/quadcopter/vel_mode (std_msgs/Bool)
  /bc_red_wp/debug_image      (sensor_msgs/Image, BGR8 with overlay)

State machine:
  FLYING      — waypoint visible, running model inference
  SEARCHING   — waypoint left FOV, rotating slowly toward disappearance side
                  if waypoint reappears → FLYING
                  if timeout (search_timeout_s) → GOAL_REACHED
  GOAL_REACHED — hover in place; reset to FLYING when new visible waypoint arrives
"""

import math
import os
import sys

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import cv2
import numpy as np
import rospy
import torch

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty

PROJECT_ROOT = os.path.expanduser('~/uav_kd_project')
sys.path.insert(0, PROJECT_ROOT)

from kd_uav.models.control_head import BCPolicy
from kd_uav.utils.waypoint_projection import (
    project_body_to_pixel,
    draw_red_waypoint,
    select_active_waypoint,
    IMG_W,
)

# ── States ────────────────────────────────────────────────────────────────────
S_FLYING      = 'FLYING'
S_SEARCHING   = 'SEARCHING'
S_GOAL        = 'GOAL_REACHED'


# ── Helpers ───────────────────────────────────────────────────────────────────

def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def world_to_body_2d(dx_w, dy_w, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return dx_w * c + dy_w * s, -dx_w * s + dy_w * c


def extract_body_waypoints(path_msg, uav_x, uav_y, uav_z, uav_yaw,
                            K=10, traj_look=0.15):
    waypoints = path_msg.poses
    n_wp = len(waypoints)
    if n_wp == 0:
        return None

    start_idx = max(n_wp - 1, 0)
    for idx, wp_ps in enumerate(waypoints):
        p = wp_ps.pose.position
        dx_b, _ = world_to_body_2d(p.x - uav_x, p.y - uav_y, uav_yaw)
        if dx_b >= traj_look:
            start_idx = idx
            break

    last = (uav_x, uav_y, uav_z)
    row = []
    for i in range(K):
        src = start_idx + i
        if src < n_wp:
            wp = waypoints[src].pose.position
            wx, wy, wz = wp.x, wp.y, wp.z
            last = (wx, wy, wz)
        else:
            wx, wy, wz = last
        dx_b, dy_b = world_to_body_2d(wx - uav_x, wy - uav_y, uav_yaw)
        row += [dx_b, dy_b, wz - uav_z]

    return np.array(row, dtype=np.float32)


def numpy_to_imgmsg(img_bgr):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height       = img_bgr.shape[0]
    msg.width        = img_bgr.shape[1]
    msg.encoding     = 'bgr8'
    msg.is_bigendian = 0
    msg.step         = img_bgr.shape[1] * 3
    msg.data         = img_bgr.tobytes()
    return msg


def put_text(img, text, pos=(8, 28), color=(255, 255, 0)):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,0,0), 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.65, color,   2)


# ── Policy node ───────────────────────────────────────────────────────────────

class BCRedWPPolicyNode:

    def __init__(self):
        rospy.init_node('bc_red_wp_policy_node', anonymous=False)

        model_path           = rospy.get_param('~model_path')
        hidden_dim           = rospy.get_param('~hidden_dim',         256)
        image_size           = rospy.get_param('~image_size',         224)
        pub_rate             = rospy.get_param('~pub_rate',            20)
        takeoff_delay        = rospy.get_param('~takeoff_delay',       3.0)
        self.lookahead       = rospy.get_param('~lookahead',           1.0)
        self.traj_stale      = rospy.get_param('~traj_stale_s',        2.0)
        self.K               = rospy.get_param('~K',                   10)
        self.search_yaw_rate = rospy.get_param('~search_yaw_rate',     0.3)  # rad/s
        self.search_timeout  = rospy.get_param('~search_timeout_s',    6.0)  # s

        rospy.loginfo(f"[RedWP] model        : {model_path}")
        rospy.loginfo(f"[RedWP] lookahead    : {self.lookahead} m")
        rospy.loginfo(f"[RedWP] search_yr    : {self.search_yaw_rate} rad/s")
        rospy.loginfo(f"[RedWP] search_t     : {self.search_timeout} s → GOAL")

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(f"[RedWP] device       : {self.device}")

        self.model = BCPolicy.build(
            pretrained=False, freeze_backbone=False,
            hidden_dim=hidden_dim, dropout=0.0,
        ).to(self.device)
        self.model.load_state_dict(
            torch.load(model_path, map_location=self.device)
        )
        self.model.eval()
        rospy.loginfo("[RedWP] Model loaded.")

        self.image_size  = image_size
        self.latest_img  = None
        self.latest_pose = None
        self.latest_traj = None
        self.traj_t      = None

        self.pub_cmd      = rospy.Publisher('/CERLAB/quadcopter/cmd_vel',  TwistStamped, queue_size=1)
        self.pub_takeoff  = rospy.Publisher('/CERLAB/quadcopter/takeoff',  Empty,        queue_size=1)
        self.pub_vel_mode = rospy.Publisher('/CERLAB/quadcopter/vel_mode', Bool,         queue_size=1)
        self.pub_debug    = rospy.Publisher('/bc_red_wp/debug_image',      Image,        queue_size=1)

        rospy.Subscriber('/camera/color/image_raw',
                         Image, self._image_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/CERLAB/quadcopter/pose',
                         PoseStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber('/dynamicNavigation/bspline_trajectory',
                         Path, self._traj_cb, queue_size=5)

        rospy.loginfo("[RedWP] Waiting for subscribers...")
        rospy.sleep(1.5)
        rospy.loginfo("[RedWP] Sending takeoff...")
        self.pub_takeoff.publish(Empty())
        rospy.sleep(takeoff_delay)
        rospy.loginfo("[RedWP] Enabling vel_mode...")
        self.pub_vel_mode.publish(Bool(data=True))
        rospy.sleep(0.5)
        rospy.loginfo("[RedWP] Student policy active. Waiting for 2D Nav Goal...")

        self.rate = rospy.Rate(pub_rate)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            self.latest_img = np.frombuffer(
                msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[RedWP] image_cb: {e}")

    def _pose_cb(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.latest_pose = (p.x, p.y, p.z,
                            yaw_from_quat(o.x, o.y, o.z, o.w))

    def _traj_cb(self, msg):
        self.latest_traj = msg
        self.traj_t = rospy.get_time()

    # ── Command helpers ───────────────────────────────────────────────────────

    def _pub_cmd(self, vx=0., vy=0., vz=0., yr=0.):
        cmd = TwistStamped()
        cmd.header.stamp    = rospy.Time.now()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = vx
        cmd.twist.linear.y  = vy
        cmd.twist.linear.z  = vz
        cmd.twist.angular.z = yr
        self.pub_cmd.publish(cmd)

    def _pub_debug(self, img_bgr, text, color=(255, 255, 0)):
        if self.pub_debug.get_num_connections() > 0:
            dbg = img_bgr.copy()
            put_text(dbg, text, color=color)
            self.pub_debug.publish(numpy_to_imgmsg(dbg))

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        state          = S_FLYING
        search_dir     = 1.0    # +1 = rotate left (CCW), -1 = rotate right (CW)
        search_start_t = None

        while not rospy.is_shutdown():
            if self.latest_img is None or self.latest_pose is None:
                self.rate.sleep()
                continue

            img  = self.latest_img.copy()
            pose = self.latest_pose
            now  = rospy.get_time()

            # ── Trajectory freshness ──────────────────────────────────
            traj_age = (now - self.traj_t) if self.traj_t is not None else float('inf')
            no_traj  = traj_age > self.traj_stale

            if no_traj:
                # Lost trajectory mid-search → goal reached
                if state == S_SEARCHING:
                    state = S_GOAL
                    rospy.loginfo("[RedWP] Trajectory lost during search → GOAL_REACHED")
                self._pub_cmd()
                self._pub_debug(img, "WAIT: no trajectory", color=(0, 165, 255))
                self.rate.sleep()
                continue

            # ── Compute waypoint ──────────────────────────────────────
            uav_x, uav_y, uav_z, uav_yaw = pose
            row = extract_body_waypoints(
                self.latest_traj, uav_x, uav_y, uav_z, uav_yaw,
                K=self.K, traj_look=0.15,
            )
            if row is None:
                self._pub_cmd()
                self._pub_debug(img, "WAIT: empty trajectory", color=(0, 165, 255))
                self.rate.sleep()
                continue

            dx, dy, dz, wp_idx = select_active_waypoint(row, self.lookahead, self.K)
            dist_b = float(np.sqrt(dx**2 + dy**2 + dz**2))
            u, v, visible = project_body_to_pixel(dx, dy, dz)

            # ── GOAL_REACHED ──────────────────────────────────────────
            if state == S_GOAL:
                if visible:
                    # New waypoint appeared (e.g. user sent another Nav Goal)
                    state = S_FLYING
                    rospy.loginfo("[RedWP] New waypoint visible → FLYING")
                else:
                    self._pub_cmd()
                    self._pub_debug(img, "GOAL REACHED — hover", color=(0, 255, 0))
                    rospy.loginfo_throttle(3.0, "[RedWP] GOAL_REACHED — hovering")
                    self.rate.sleep()
                    continue

            # ── SEARCHING ────────────────────────────────────────────
            if state == S_SEARCHING:
                if visible:
                    state = S_FLYING
                    rospy.loginfo("[RedWP] Waypoint found → FLYING")
                else:
                    elapsed = now - search_start_t
                    if elapsed >= self.search_timeout:
                        state = S_GOAL
                        rospy.loginfo(
                            f"[RedWP] Search {elapsed:.1f}s, no waypoint → GOAL_REACHED")
                        self.rate.sleep()
                        continue

                    # Rotate toward the side the dot disappeared
                    yr = self.search_yaw_rate * search_dir
                    self._pub_cmd(yr=yr)
                    side = "LEFT" if search_dir > 0 else "RIGHT"
                    self._pub_debug(img,
                        f"SEARCHING {side} {elapsed:.1f}/{self.search_timeout:.0f}s",
                        color=(0, 165, 255))
                    rospy.loginfo_throttle(2.0,
                        f"[RedWP] SEARCHING {side}  yr={yr:.2f}  t={elapsed:.1f}s")
                    self.rate.sleep()
                    continue

            # ── FLYING ────────────────────────────────────────────────
            # state is S_FLYING here (either was already, or just transitioned)
            if not visible:
                # Waypoint just left the FOV — determine which side
                search_dir = 1.0 if u < 0 else -1.0   # u<0 → left, u≥IMG_W → right
                search_start_t = now
                state = S_SEARCHING
                side = "LEFT" if search_dir > 0 else "RIGHT"
                rospy.loginfo(
                    f"[RedWP] Waypoint lost (u={u:.0f}) → SEARCHING {side}")
                yr = self.search_yaw_rate * search_dir
                self._pub_cmd(yr=yr)
                self._pub_debug(img, f"SEARCHING {side} 0/{self.search_timeout:.0f}s",
                                color=(0, 165, 255))
                self.rate.sleep()
                continue

            # Waypoint visible: run model
            annotated = draw_red_waypoint(img, u, v, dist_b)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            rgb = cv2.resize(rgb, (self.image_size, self.image_size),
                             interpolation=cv2.INTER_LINEAR)
            t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
            t = t.unsqueeze(0).to(self.device)

            with torch.no_grad():
                pred = self.model(t).squeeze(0).cpu().numpy()

            vx_b, vy_b, vz_b, yaw_rate = (
                float(pred[0]), float(pred[1]), float(pred[2]), float(pred[3]))

            self._pub_cmd(vx_b, vy_b, vz_b, yaw_rate)
            rospy.loginfo_throttle(2.0,
                f"[RedWP] {state} wp{wp_idx} d={dist_b:.2f}m "
                f"u={u:.0f},v={v:.0f} | "
                f"vx={vx_b:.3f} vy={vy_b:.3f} yr={yaw_rate:.3f}")

            self._pub_debug(annotated,
                f"wp{wp_idx} d={dist_b:.1f}m  vx={vx_b:.2f} yr={yaw_rate:.2f}")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = BCRedWPPolicyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
