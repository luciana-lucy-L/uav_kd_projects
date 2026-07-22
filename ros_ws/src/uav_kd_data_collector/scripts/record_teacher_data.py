#!/usr/bin/env python3
"""
UAV KD Teacher Demonstration Data Collector

Subscribes (read-only) to CERLAB teacher topics and writes per-episode:
  images/NNNNNN.png
  ctrl_world.csv  — teacher velocity in world frame + yaw_rate
  ctrl_body.csv   — teacher velocity rotated to UAV body frame + yaw_rate
  traj_world.csv  — K future B-spline waypoints in world frame (absolute)
  traj_body.csv   — K future waypoints in UAV body frame (relative)
  odom.csv        — UAV odometry
  pose.csv        — UAV pose + extracted yaw
  metadata.yaml   — written on shutdown

Rosbag is recorded by a separate node launched from record_teacher.launch.

Coordinate conventions
----------------------
  World frame : ROS standard (x-East-ish, y-North-ish, z-up)
  Body frame  : x-forward, y-left, z-up
  yaw         : angle from world-x to body-x, counterclockwise (rad)
  Rotation from world to body: [vx_b, vy_b] = R(-yaw) @ [vx_w, vy_w]

yaw_rate computation
--------------------
  Computed from consecutive /autonomous_flight/target_state yaw values.
  target_state.yaw is the desired yaw in world frame.
  yaw_rate = wrap(yaw[t] - yaw[t-1]) / dt   (rad/s)
  Using target_state timestamps (wall clock fallback if stamp is zero).

DOES NOT modify any CERLAB or uav_simulator source files.
"""

import os
import csv
import math
import yaml
import sys

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

try:
    from tracking_controller.msg import Target
except ImportError:
    print("[DataCollector] FATAL: Cannot import tracking_controller.msg.Target.\n"
          "  Make sure cerlab_ws is sourced:\n"
          "  source /home/l/cerlab_ws/devel/setup.bash", file=sys.stderr)
    sys.exit(1)


# ── Pure math helpers ──────────────────────────────────────────────────────────

def yaw_from_quat(qx, qy, qz, qw):
    """Extract yaw (Z-rotation) from quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_angle(a):
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def world_to_body_2d(vx_w, vy_w, yaw):
    """
    Rotate a 2D world-frame vector into the UAV body frame.
    Body frame: x-forward, y-left.
    vx_b =  vx_w * cos(yaw) + vy_w * sin(yaw)
    vy_b = -vx_w * sin(yaw) + vy_w * cos(yaw)
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return vx_w * c + vy_w * s, -vx_w * s + vy_w * c


# ── Main collector class ───────────────────────────────────────────────────────

class TeacherDataCollector:

    STALE_S = 1.0   # seconds: skip frame if any topic older than this

    def __init__(self):
        rospy.init_node('uav_kd_data_collector', anonymous=False)

        # ── Parameters ──────────────────────────────────────────────
        self.episode_id  = rospy.get_param('~episode_id',      'ep_default')
        self.output_dir  = rospy.get_param('~output_dir',
                                           '/home/l/uav_kd_project/data/raw')
        self.world       = rospy.get_param('~world',           'unknown')
        self.K            = int(rospy.get_param('~K',              10))
        self.traj_look    = float(rospy.get_param('~traj_lookahead', 0.15))
        self.sync_tol     = float(rospy.get_param('~sync_tolerance', 1.0))
        self.downsample   = bool(rospy.get_param('~downsample',      False))
        self.ds_step      = int(rospy.get_param('~downsample_step',  3))

        # ── Directories ─────────────────────────────────────────────
        self.ep_dir  = os.path.join(self.output_dir, self.episode_id)
        self.img_dir = os.path.join(self.ep_dir, 'images')
        self.bag_dir = os.path.join(self.ep_dir, 'rosbag')
        for d in [self.img_dir, self.bag_dir]:
            os.makedirs(d, exist_ok=True)

        rospy.loginfo(
            f"[Collector] episode={self.episode_id}  K={self.K}  "
            f"sync_tol={self.sync_tol}s  downsample={self.downsample}"
            f"  dir={self.ep_dir}"
        )

        # ── State ───────────────────────────────────────────────────
        self.bridge = CvBridge()

        # Latest message + wall-clock receive time for each topic
        self._target   = None;  self._target_t  = None
        self._odom     = None;  self._odom_t    = None
        self._pose     = None;  self._pose_t    = None
        self._traj     = None;  self._traj_t    = None

        # yaw_rate computation (from consecutive target_state.yaw values)
        self._prev_yaw   = None
        self._prev_yaw_t = None
        self._yaw_rate   = 0.0

        self._frame_count = 0   # number of frames successfully saved
        self._tick        = 0   # raw image callback counter (for downsampling)
        self._shutdown    = False  # set True before closing files to guard callbacks

        # ── CSV files ────────────────────────────────────────────────
        self._fds      = {}
        self._writers  = {}
        self._open_csv_files()

        # ── Subscribers ─────────────────────────────────────────────
        # target_state at ~200 Hz; large queue to avoid dropping messages
        # used for yaw_rate computation.
        rospy.Subscriber('/autonomous_flight/target_state',
                         Target, self._on_target, queue_size=200)
        rospy.Subscriber('/CERLAB/quadcopter/odom',
                         Odometry, self._on_odom, queue_size=50)
        rospy.Subscriber('/CERLAB/quadcopter/pose',
                         PoseStamped, self._on_pose, queue_size=50)
        rospy.Subscriber('/dynamicNavigation/bspline_trajectory',
                         Path, self._on_traj, queue_size=50)
        # Image callback is last — it is the master trigger for saving.
        rospy.Subscriber('/camera/color/image_raw',
                         Image, self._on_image, queue_size=5)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("[Collector] Subscribed. Waiting for all topics before saving...")
        rospy.loginfo("[Collector] Note: bspline_trajectory only publishes after a 2D Nav Goal is sent.")

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _on_target(self, msg):
        now = rospy.get_time()
        self._target   = msg
        self._target_t = now

        # Compute yaw_rate from consecutive yaw values.
        # target_state.yaw is the desired heading in world frame (rad).
        yaw = msg.yaw
        if self._prev_yaw is not None and self._prev_yaw_t is not None:
            dt = now - self._prev_yaw_t
            if dt > 1e-6:
                self._yaw_rate = wrap_angle(yaw - self._prev_yaw) / dt
        self._prev_yaw   = yaw
        self._prev_yaw_t = now

    def _on_odom(self, msg):
        self._odom   = msg
        self._odom_t = rospy.get_time()

    def _on_pose(self, msg):
        self._pose   = msg
        self._pose_t = rospy.get_time()

    def _on_traj(self, msg):
        self._traj   = msg
        self._traj_t = rospy.get_time()

    # ── Image callback (master trigger) ──────────────────────────────────────

    def _on_image(self, msg):
        if self._shutdown:
            return

        self._tick += 1

        # Optional frame skip
        if self.downsample and (self._tick % self.ds_step != 0):
            return

        # Require all topics to have recent data
        if not self._all_ready():
            return

        fid = self._frame_count
        self._frame_count += 1
        t_now = rospy.get_time()
        ts = f'{t_now:.6f}'

        # ── Save image ───────────────────────────────────────────────
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(self.img_dir, f'{fid:06d}.png'), img)
        except Exception as exc:
            rospy.logwarn(f"[Collector] Image write failed (frame {fid}): {exc}")
            self._frame_count -= 1
            return

        # Snapshot cached data (avoid mutation during processing)
        target = self._target
        odom   = self._odom
        pose   = self._pose
        traj   = self._traj

        # ── UAV state from pose ──────────────────────────────────────
        pos = pose.pose.position
        ori = pose.pose.orientation
        uav_x   = pos.x
        uav_y   = pos.y
        uav_z   = pos.z
        uav_yaw = yaw_from_quat(ori.x, ori.y, ori.z, ori.w)

        # ── odom.csv ─────────────────────────────────────────────────
        op = odom.pose.pose.position
        ov = odom.twist.twist.linear
        self._write('odom', [
            fid, ts,
            f'{op.x:.6f}', f'{op.y:.6f}', f'{op.z:.6f}',
            f'{ov.x:.6f}', f'{ov.y:.6f}', f'{ov.z:.6f}',
        ])

        # ── pose.csv ─────────────────────────────────────────────────
        self._write('pose', [
            fid, ts,
            f'{uav_x:.6f}', f'{uav_y:.6f}', f'{uav_z:.6f}',
            f'{ori.x:.6f}', f'{ori.y:.6f}', f'{ori.z:.6f}', f'{ori.w:.6f}',
            f'{uav_yaw:.6f}',
        ])

        # ── ctrl_world.csv ───────────────────────────────────────────
        # target_state.velocity is in world frame
        vx_w   = target.velocity.x
        vy_w   = target.velocity.y
        vz     = target.velocity.z
        yr     = self._yaw_rate      # rad/s, computed from consecutive yaw
        t_yaw  = target.yaw          # desired heading (world frame)

        self._write('ctrl_world', [
            fid, ts,
            f'{vx_w:.6f}', f'{vy_w:.6f}', f'{vz:.6f}',
            f'{yr:.6f}', f'{t_yaw:.6f}',
        ])

        # ── ctrl_body.csv ────────────────────────────────────────────
        # Rotate world-frame velocity into UAV body frame using actual pose yaw.
        # yaw_rate is a scalar — same value in both frames.
        vx_b, vy_b = world_to_body_2d(vx_w, vy_w, uav_yaw)

        self._write('ctrl_body', [
            fid, ts,
            f'{vx_b:.6f}', f'{vy_b:.6f}', f'{vz:.6f}',
            f'{yr:.6f}', f'{t_yaw:.6f}',
        ])

        # ── traj_world.csv and traj_body.csv ─────────────────────────
        # bspline_trajectory publishes the FULL path from start to goal.
        # The UAV may have already passed the early waypoints, so we must
        # find the closest waypoint to the current UAV position and extract
        # K points starting from there (i.e. future waypoints only).
        waypoints = traj.poses        # list of geometry_msgs/PoseStamped
        n_wp      = len(waypoints)
        row_w = [fid, ts]
        row_b = [fid, ts]

        # Find the start index for future waypoints.
        # Strategy: scan from the beginning and take the first waypoint whose
        # body-frame x offset (dx_b) is >= traj_lookahead.
        # This skips waypoints that are at or behind the UAV, and ensures
        # the first extracted waypoint is meaningfully ahead.
        # Fallback: if no waypoint satisfies the condition, use the last one.
        start_idx = max(n_wp - 1, 0)
        for idx, wp_ps in enumerate(waypoints):
            p = wp_ps.pose.position
            dx_w = p.x - uav_x
            dy_w = p.y - uav_y
            dx_b, _ = world_to_body_2d(dx_w, dy_w, uav_yaw)
            if dx_b >= self.traj_look:
                start_idx = idx
                break

        # Extract K future waypoints starting from start_idx.
        # Pad with the last available point if fewer than K remain.
        last = (uav_x, uav_y, uav_z)   # fallback if trajectory is empty
        for i in range(self.K):
            src = start_idx + i
            if src < n_wp:
                wp = waypoints[src].pose.position
                wx, wy, wz = wp.x, wp.y, wp.z
                last = (wx, wy, wz)
            else:
                wx, wy, wz = last

            # World-frame absolute coordinates
            row_w += [f'{wx:.6f}', f'{wy:.6f}', f'{wz:.6f}']

            # Body-frame relative coordinates
            dx_w = wx - uav_x
            dy_w = wy - uav_y
            dz   = wz - uav_z          # z component is same in both frames
            dx_b, dy_b = world_to_body_2d(dx_w, dy_w, uav_yaw)
            row_b += [f'{dx_b:.6f}', f'{dy_b:.6f}', f'{dz:.6f}']

        self._write('traj_world', row_w)
        self._write('traj_body',  row_b)

        # Progress log (every 30 frames ≈ 1 second at 30 Hz)
        if fid % 30 == 0:
            rospy.loginfo(
                f"[Collector] frame {fid:05d} | "
                f"vx_w={vx_w:+.2f} vx_b={vx_b:+.2f} yr={yr:+.3f} "
                f"poses={n_wp} z={uav_z:.2f}"
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _all_ready(self):
        """Return True if every topic has been heard within STALE_S seconds."""
        now = rospy.get_time()
        checks = [
            ('target_state',        self._target_t),
            ('odom',                self._odom_t),
            ('pose',                self._pose_t),
            ('bspline_trajectory',  self._traj_t),
        ]
        for name, t in checks:
            if t is None:
                rospy.loginfo_throttle(
                    5.0, f"[Collector] Waiting for first message on {name} ...")
                return False
            age = now - t
            if age > self.sync_tol:
                rospy.logwarn_throttle(
                    5.0, f"[Collector] {name} stale ({age:.2f}s > {self.sync_tol}s)")
                return False
        return True

    def _open_csv_files(self):
        K = self.K
        # Build column headers for trajectory CSVs
        traj_w_hdr = (['frame_id', 'timestamp'] +
                       [f'{ax}{i}' for i in range(K) for ax in ('x', 'y', 'z')])
        traj_b_hdr = (['frame_id', 'timestamp'] +
                       [f'd{ax}{i}' for i in range(K) for ax in ('x', 'y', 'z')])

        specs = {
            'ctrl_world': ['frame_id', 'timestamp',
                           'vx_w', 'vy_w', 'vz_w', 'yaw_rate', 'target_yaw'],
            'ctrl_body':  ['frame_id', 'timestamp',
                           'vx_b', 'vy_b', 'vz_b', 'yaw_rate', 'target_yaw'],
            'traj_world': traj_w_hdr,
            'traj_body':  traj_b_hdr,
            'odom':       ['frame_id', 'timestamp',
                           'x', 'y', 'z', 'vx', 'vy', 'vz'],
            'pose':       ['frame_id', 'timestamp',
                           'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'],
        }

        for name, headers in specs.items():
            path = os.path.join(self.ep_dir, f'{name}.csv')
            fd   = open(path, 'w', newline='')
            w    = csv.writer(fd)
            w.writerow(headers)
            self._fds[name]     = fd
            self._writers[name] = w

    def _write(self, name, row):
        if self._shutdown:
            return
        self._writers[name].writerow(row)

    def _on_shutdown(self):
        self._shutdown = True   # stop callbacks from writing to closing files
        for fd in self._fds.values():
            fd.flush()
            fd.close()

        rospy.loginfo(
            f"[Collector] Shutdown — {self._frame_count} frames saved to {self.ep_dir}")

        meta = {
            'episode_id':        self.episode_id,
            'world':             self.world,
            'teacher_system':    'CERLAB Autonomous Flight / dynamic_navigation',
            'student_used':      False,
            'record_type':       'teacher_demonstration',
            'K':                 self.K,
            'sync_tolerance_s':  self.sync_tol,
            'downsample':        self.downsample,
            'downsample_step':   self.ds_step if self.downsample else 'N/A',
            'camera_topic':      '/camera/color/image_raw',
            'control_topic':     '/autonomous_flight/target_state',
            'trajectory_topic':  '/dynamicNavigation/bspline_trajectory',
            'odom_topic':        '/CERLAB/quadcopter/odom',
            'pose_topic':        '/CERLAB/quadcopter/pose',
            'total_frames':      self._frame_count,
            'result': {
                'success':   None,
                'collision': None,
                'notes':     'Fill in after visual inspection.',
            },
        }

        meta_path = os.path.join(self.ep_dir, 'metadata.yaml')
        with open(meta_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False, sort_keys=False)

        rospy.loginfo(f"[Collector] metadata.yaml written → {meta_path}")


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    try:
        TeacherDataCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
