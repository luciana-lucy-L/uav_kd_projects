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


class BCPolicy(nn.Module):
    def __init__(self, state_dim=13, action_dim=3, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
        )

    def forward(self, x):
        return self.net(x)


class BCPolicyVelNode:
    def __init__(self):
        rospy.loginfo("Starting BCPolicyVelNode (TwistStamped)")

        model_path = os.path.expanduser("~/uav_bc_training/models/bc_policy_vel_kd.pt")
        if not os.path.exists(model_path):
            rospy.logerr("Model not found: %s", model_path)
            raise RuntimeError("Model not found")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo("Using device: %s", self.device)

        self.model = BCPolicy().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        self.last_odom = None

        rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self.odom_cb, queue_size=1)

        # 注意：发布 TwistStamped
        self.pub_cmd = rospy.Publisher("/CERLAB/quadcopter/cmd_vel", TwistStamped, queue_size=10)

        self.pub_takeoff = rospy.Publisher("/CERLAB/quadcopter/takeoff", Empty, queue_size=1)
        self.pub_vel_mode = rospy.Publisher("/CERLAB/quadcopter/vel_mode", Bool, queue_size=1)

        self.rate = rospy.Rate(20)

        rospy.sleep(1.0)
        self.pub_takeoff.publish(Empty())
        rospy.sleep(1.0)
        self.pub_vel_mode.publish(Bool(data=True))

        rospy.loginfo("Takeoff + vel_mode done. BC velocity control active.")

    def odom_cb(self, msg):
        self.last_odom = msg

    def build_state(self, odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular

        state = np.array([
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z,
        ], dtype=np.float32)

        return torch.from_numpy(state).unsqueeze(0).to(self.device)

    def run(self):
        rospy.loginfo("Publishing TwistStamped cmd_vel from BC...")
        while not rospy.is_shutdown():
            if self.last_odom is not None:
                s = self.build_state(self.last_odom)
                with torch.no_grad():
                    v = self.model(s).squeeze(0).cpu().numpy()

                cmd = TwistStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = "map"  # 随便填，通常不影响控制

                cmd.twist.linear.x = float(v[0])
                cmd.twist.linear.y = float(v[1])
                cmd.twist.linear.z = float(v[2])

                # 不控制角速度就置 0
                cmd.twist.angular.x = 0.0
                cmd.twist.angular.y = 0.0
                cmd.twist.angular.z = 0.0

                self.pub_cmd.publish(cmd)

            self.rate.sleep()


def main():
    rospy.init_node("bc_policy_node_vel")
    node = BCPolicyVelNode()
    node.run()


if __name__ == "__main__":
    main()
