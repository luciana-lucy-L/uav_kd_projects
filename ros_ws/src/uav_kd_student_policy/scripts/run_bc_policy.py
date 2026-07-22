#!/home/l/miniconda3/envs/dbc241/bin/python3
"""
run_bc_policy.py — BC student policy deployment node

Subscribes : /camera/color/image_raw  (sensor_msgs/Image, BGR8)
Publishes  : /CERLAB/quadcopter/cmd_vel  (geometry_msgs/TwistStamped)
             /CERLAB/quadcopter/takeoff  (std_msgs/Empty)
             /CERLAB/quadcopter/vel_mode (std_msgs/Bool)

Control output is body-frame [vx_b, vy_b, vz_b, yaw_rate], which the
CERLAB quadcopter plugin accepts directly via cmd_vel.
"""

import os
import sys
import time

# Inject ROS Python packages so dbc241 conda python can import rospy
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import cv2
import numpy as np
import rospy
import torch

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty

# Allow importing kd_uav from the project root
PROJECT_ROOT = os.path.expanduser('~/uav_kd_project')
sys.path.insert(0, PROJECT_ROOT)
from kd_uav.models.control_head import BCPolicy


class BCPolicyNode:
    def __init__(self):
        rospy.init_node('bc_policy_node', anonymous=False)

        model_path    = rospy.get_param('~model_path')
        hidden_dim    = rospy.get_param('~hidden_dim',    256)
        image_size    = rospy.get_param('~image_size',    224)
        pub_rate      = rospy.get_param('~pub_rate',      20)
        takeoff_delay = rospy.get_param('~takeoff_delay', 3.0)

        rospy.loginfo(f"[BC] model_path    : {model_path}")
        rospy.loginfo(f"[BC] hidden_dim    : {hidden_dim}")
        rospy.loginfo(f"[BC] image_size    : {image_size}")
        rospy.loginfo(f"[BC] pub_rate      : {pub_rate} Hz")
        rospy.loginfo(f"[BC] takeoff_delay : {takeoff_delay} s")

        # ── Device & model ────────────────────────────────────────────────────
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(f"[BC] device        : {self.device}")

        self.model = BCPolicy.build(
            pretrained=False,
            freeze_backbone=False,
            hidden_dim=hidden_dim,
            dropout=0.0,
        ).to(self.device)
        self.model.load_state_dict(
            torch.load(model_path, map_location=self.device)
        )
        self.model.eval()
        rospy.loginfo("[BC] Model loaded and ready.")

        self.image_size = image_size
        self.latest_img = None   # (3, H, W) float32 tensor on device, or None

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_cmd      = rospy.Publisher('/CERLAB/quadcopter/cmd_vel',  TwistStamped, queue_size=1)
        self.pub_takeoff  = rospy.Publisher('/CERLAB/quadcopter/takeoff',  Empty,        queue_size=1)
        self.pub_vel_mode = rospy.Publisher('/CERLAB/quadcopter/vel_mode', Bool,         queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber('/camera/color/image_raw', Image, self._image_cb, queue_size=1, buff_size=2**24)

        # ── Takeoff sequence ─────────────────────────────────────────────────
        rospy.loginfo("[BC] Waiting for subscribers to connect...")
        rospy.sleep(1.5)
        rospy.loginfo("[BC] Sending takeoff...")
        self.pub_takeoff.publish(Empty())
        rospy.sleep(takeoff_delay)
        rospy.loginfo("[BC] Enabling vel_mode...")
        self.pub_vel_mode.publish(Bool(data=True))
        rospy.sleep(0.5)
        rospy.loginfo("[BC] Student policy active.")

        self.rate = rospy.Rate(pub_rate)

    # ── Image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            # ROS Image (BGR8) → numpy (H, W, 3) uint8
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (self.image_size, self.image_size), interpolation=cv2.INTER_LINEAR)
            t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0  # (3, H, W)
            self.latest_img = t.unsqueeze(0).to(self.device)             # (1, 3, H, W)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[BC] image_cb error: {e}")

    # ── Control loop ──────────────────────────────────────────────────────────

    def run(self):
        rospy.loginfo("[BC] Publishing cmd_vel...")
        while not rospy.is_shutdown():
            if self.latest_img is not None:
                with torch.no_grad():
                    pred = self.model(self.latest_img).squeeze(0).cpu().numpy()

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
                    f"[BC] vx={vx_b:.3f}  vy={vy_b:.3f}  vz={vz_b:.3f}  yr={yaw_rate:.3f}")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = BCPolicyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
