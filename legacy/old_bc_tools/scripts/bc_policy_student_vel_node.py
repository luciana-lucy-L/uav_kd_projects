#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np

import torch
import torch.nn as nn

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty, Bool


# ============================================================
# Student-only BC Policy
# ============================================================
class BCPolicyStudent(nn.Module):
    """
    KD student policy
    NOTE: hidden_dim must match training exactly
    """
    def __init__(self, state_dim=13, action_dim=3, hidden_dim=32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# ROS Node
# ============================================================
class BCPolicyStudentVelNode:
    def __init__(self):
        rospy.loginfo("Starting BCPolicyStudentVelNode (KD Student)")

        # ---------- paths (explicit & independent) ----------
        self.model_path = os.path.expanduser(
            "~/uav_bc_training/models/bc_policy_vel_kd.pt"
        )
        self.stats_path = os.path.expanduser(
            "~/uav_bc_training/models/bc_vel_stats.npz"
        )

        if not os.path.exists(self.model_path):
            rospy.logerr("Student model not found: %s", self.model_path)
            raise RuntimeError("Student model not found")

        if not os.path.exists(self.stats_path):
            rospy.logerr("Stats not found: %s", self.stats_path)
            raise RuntimeError("Stats not found")

        # ---------- device ----------
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo("Using device: %s", self.device)

        # ---------- load student model ----------
        self.model = BCPolicyStudent(hidden_dim=32).to(self.device)
        self.model.load_state_dict(
            torch.load(self.model_path, map_location=self.device)
        )
        self.model.eval()

        rospy.loginfo("Loaded KD student model: %s", self.model_path)

        # ---------- load normalization stats ----------
        stats = np.load(self.stats_path)
        self.state_mean = torch.from_numpy(
            stats["state_mean"]
        ).float().to(self.device)
        self.state_std = torch.from_numpy(
            stats["state_std"]
        ).float().to(self.device)

        self.action_mean = stats["action_mean"].astype(np.float32)
        self.action_std = stats["action_std"].astype(np.float32)

        rospy.loginfo("Loaded normalization stats")

        # ---------- params ----------
        self.v_max = rospy.get_param("~v_max", 1.0)
        self.rate = rospy.Rate(20)

        # ---------- ROS I/O ----------
        self.last_odom = None
        rospy.Subscriber(
            "/CERLAB/quadcopter/odom",
            Odometry,
            self.odom_cb,
            queue_size=1,
        )

        self.pub_cmd = rospy.Publisher(
            "/CERLAB/quadcopter/cmd_vel",
            TwistStamped,
            queue_size=10,
        )
        self.pub_takeoff = rospy.Publisher(
            "/CERLAB/quadcopter/takeoff",
            Empty,
            queue_size=1,
        )
        self.pub_vel_mode = rospy.Publisher(
            "/CERLAB/quadcopter/vel_mode",
            Bool,
            queue_size=1,
        )

        rospy.sleep(1.0)
        self.pub_takeoff.publish(Empty())
        rospy.sleep(1.0)
        self.pub_vel_mode.publish(Bool(data=True))

        rospy.loginfo("Takeoff + vel_mode enabled. KD student active.")

    # --------------------------------------------------------
    def odom_cb(self, msg):
        self.last_odom = msg

    # --------------------------------------------------------
    def build_state(self, odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular

        state = np.array(
            [
                p.x, p.y, p.z,
                q.x, q.y, q.z, q.w,
                lv.x, lv.y, lv.z,
                av.x, av.y, av.z,
            ],
            dtype=np.float32,
        )

        return torch.from_numpy(state).unsqueeze(0).to(self.device)

    # --------------------------------------------------------
    def run(self):
        rospy.loginfo("Publishing cmd_vel from KD student policy...")
        while not rospy.is_shutdown():
            if self.last_odom is not None:
                s = self.build_state(self.last_odom)

                # normalize
                s_norm = (s - self.state_mean) / self.state_std

                with torch.no_grad():
                    a_norm = (
                        self.model(s_norm)
                        .squeeze(0)
                        .cpu()
                        .numpy()
                    )

                # denormalize
                v = a_norm * self.action_std + self.action_mean
                v = np.clip(v, -self.v_max, self.v_max)

                cmd = TwistStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = "map"

                cmd.twist.linear.x = float(v[0])
                cmd.twist.linear.y = float(v[1])
                cmd.twist.linear.z = float(v[2])

                cmd.twist.angular.x = 0.0
                cmd.twist.angular.y = 0.0
                cmd.twist.angular.z = 0.0

                self.pub_cmd.publish(cmd)

            self.rate.sleep()


# ============================================================
def main():
    rospy.init_node("bc_policy_student_vel_node")
    node = BCPolicyStudentVelNode()
    node.run()


if __name__ == "__main__":
    main()
