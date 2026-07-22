#!/home/l/miniconda3/envs/dbc241/bin/python3
"""
run_self_red_wp_policy.py — SelfRedWP student policy deployment node

Subscribes : /camera/color/image_raw  (sensor_msgs/Image, BGR8)
Publishes  : /CERLAB/quadcopter/cmd_vel  (geometry_msgs/TwistStamped)
             /CERLAB/quadcopter/takeoff  (std_msgs/Empty)
             /CERLAB/quadcopter/vel_mode (std_msgs/Bool)

Per frame: predict_traj(raw image) -> select active waypoint -> project to
pixel -> draw red dot on the ORIGINAL-resolution image (matching training,
where the pinhole projection is calibrated for 640x480, not the resized
training resolution) -> resize -> predict_ctrl(annotated image) -> cmd_vel.

Always self-predicted (no ground truth exists at inference) — this is the
p_self=1.0 condition the training curriculum was built to prepare for.
Fully self-contained: unlike BC-RedWP, no live planner is needed.

Control output is body-frame [vx_b, vy_b, vz_b, yaw_rate], which the
CERLAB quadcopter plugin accepts directly via cmd_vel.
"""

import os
import sys
import time

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import cv2
import numpy as np
import rospy
import torch

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty

PROJECT_ROOT = os.path.expanduser('~/uav_kd_project')
sys.path.insert(0, PROJECT_ROOT)
from kd_uav.models.self_red_wp_policy import SelfRedWPPolicy
from kd_uav.utils.waypoint_projection import (
    select_active_waypoint, project_body_to_pixel, draw_red_waypoint,
)


class SelfRedWPPolicyNode:
    def __init__(self):
        rospy.init_node('self_red_wp_policy_node', anonymous=False)

        model_path    = rospy.get_param('~model_path')
        hidden_dim    = rospy.get_param('~hidden_dim',    256)
        K             = rospy.get_param('~K',             10)
        image_size    = rospy.get_param('~image_size',    224)
        lookahead     = rospy.get_param('~lookahead',     1.0)
        pub_rate      = rospy.get_param('~pub_rate',      20)
        takeoff_delay = rospy.get_param('~takeoff_delay', 3.0)

        rospy.loginfo(f"[SelfRedWP] model_path    : {model_path}")
        rospy.loginfo(f"[SelfRedWP] hidden_dim    : {hidden_dim}")
        rospy.loginfo(f"[SelfRedWP] K             : {K}")
        rospy.loginfo(f"[SelfRedWP] image_size    : {image_size}")
        rospy.loginfo(f"[SelfRedWP] lookahead     : {lookahead} m")
        rospy.loginfo(f"[SelfRedWP] pub_rate      : {pub_rate} Hz")
        rospy.loginfo(f"[SelfRedWP] takeoff_delay : {takeoff_delay} s")

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(f"[SelfRedWP] device        : {self.device}")

        self.model = SelfRedWPPolicy.build(
            pretrained=False,
            freeze_backbone=False,
            hidden_dim=hidden_dim,
            dropout=0.0,
            K=K,
        ).to(self.device)
        self.model.load_state_dict(
            torch.load(model_path, map_location=self.device)
        )
        self.model.eval()
        rospy.loginfo("[SelfRedWP] Model loaded and ready. (self-predicted waypoint, no live planner needed)")

        self.image_size = image_size
        self.K          = K
        self.lookahead  = lookahead
        self.latest_raw_bgr = None   # (H, W, 3) uint8 BGR, original resolution, or None

        self.pub_cmd      = rospy.Publisher('/CERLAB/quadcopter/cmd_vel',  TwistStamped, queue_size=1)
        self.pub_takeoff  = rospy.Publisher('/CERLAB/quadcopter/takeoff',  Empty,        queue_size=1)
        self.pub_vel_mode = rospy.Publisher('/CERLAB/quadcopter/vel_mode', Bool,         queue_size=1)

        rospy.Subscriber('/camera/color/image_raw', Image, self._image_cb, queue_size=1, buff_size=2**24)

        rospy.loginfo("[SelfRedWP] Waiting for subscribers to connect...")
        rospy.sleep(1.5)
        rospy.loginfo("[SelfRedWP] Sending takeoff...")
        self.pub_takeoff.publish(Empty())
        rospy.sleep(takeoff_delay)
        rospy.loginfo("[SelfRedWP] Enabling vel_mode...")
        self.pub_vel_mode.publish(Bool(data=True))
        rospy.sleep(0.5)
        rospy.loginfo("[SelfRedWP] Student policy active.")

        self.rate = rospy.Rate(pub_rate)

    def _image_cb(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            self.latest_raw_bgr = img.copy()   # (H, W, 3) uint8 BGR, original resolution
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[SelfRedWP] image_cb error: {e}")

    def _to_tensor(self, bgr_img):
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.image_size, self.image_size), interpolation=cv2.INTER_LINEAR)
        t = torch.from_numpy(resized).permute(2, 0, 1).float() / 255.0
        return t.unsqueeze(0).to(self.device)

    def run(self):
        rospy.loginfo("[SelfRedWP] Publishing cmd_vel...")
        while not rospy.is_shutdown():
            if self.latest_raw_bgr is not None:
                raw_bgr = self.latest_raw_bgr

                with torch.no_grad():
                    # Pass 1: predict trajectory from raw image
                    img_t = self._to_tensor(raw_bgr)
                    traj_pred = self.model.predict_traj(img_t).squeeze(0).cpu().numpy()  # (K, 3)

                    # Select active waypoint, project, draw on ORIGINAL resolution image
                    dx, dy, dz, _ = select_active_waypoint(traj_pred.reshape(-1), self.lookahead, self.K)
                    u, v, visible = project_body_to_pixel(dx, dy, dz)
                    if visible:
                        dist_b = float(np.sqrt(dx**2 + dy**2 + dz**2))
                        annotated_bgr = draw_red_waypoint(raw_bgr, u, v, dist_b)
                    else:
                        annotated_bgr = raw_bgr.copy()

                    # Pass 2: predict control from annotated image
                    annotated_t = self._to_tensor(annotated_bgr)
                    pred = self.model.predict_ctrl(annotated_t).squeeze(0).cpu().numpy()

                vx_b, vy_b, vz_b, yaw_rate = float(pred[0]), float(pred[1]), float(pred[2]), float(pred[3])

                cmd = TwistStamped()
                cmd.header.stamp    = rospy.Time.now()
                cmd.header.frame_id = 'base_link'
                cmd.twist.linear.x  = vx_b
                cmd.twist.linear.y  = vy_b
                cmd.twist.linear.z  = vz_b
                cmd.twist.angular.z = yaw_rate

                self.pub_cmd.publish(cmd)

                rospy.loginfo_throttle(2.0,
                    f"[SelfRedWP] vx={vx_b:.3f}  vy={vy_b:.3f}  vz={vz_b:.3f}  yr={yaw_rate:.3f}  "
                    f"wp=({dx:.2f},{dy:.2f},{dz:.2f}) visible={visible}")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = SelfRedWPPolicyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
