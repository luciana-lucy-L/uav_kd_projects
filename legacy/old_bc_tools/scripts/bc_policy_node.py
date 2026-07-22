#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np

import torch
import torch.nn as nn

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
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


class BCPolicyNode(object):
    def __init__(self):
        # ========================
        # 1. 加载模型
        # ========================
        default_model = os.path.expanduser('~/uav_bc_training/models/bc_policy.pt')
        model_path = rospy.get_param('~model_path', default_model)

        if not os.path.exists(model_path):
            rospy.logerr("Model file not found: %s", model_path)
            raise RuntimeError("Model file not found")

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo("Using device: %s", device)
        self.device = device

        self.model = BCPolicy(state_dim=13, action_dim=3, hidden_dim=64).to(self.device)
        state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state_dict)
        self.model.eval()
        rospy.loginfo("Loaded BC policy from %s", model_path)

        # ========================
        # 2. ROS 通信
        # ========================
        self.last_odom = None

        self.sub_odom = rospy.Subscriber(
            '/CERLAB/quadcopter/odom',
            Odometry,
            self.odom_cb,
            queue_size=1
        )

        # 控制输出（加速度）
        self.pub_cmd = rospy.Publisher(
            '/CERLAB/quadcopter/cmd_acc',
            Vector3,
            queue_size=10
        )

        # 起飞 & 模式切换
        self.pub_takeoff = rospy.Publisher(
            '/CERLAB/quadcopter/takeoff',
            Empty,
            queue_size=1
        )

        self.pub_vel_mode = rospy.Publisher(
            '/CERLAB/quadcopter/vel_mode',
            Bool,
            queue_size=1
        )

        rate_hz = rospy.get_param('~rate', 20.0)
        self.rate = rospy.Rate(rate_hz)

        # ========================
        # 3. 自动起飞 + 切模式（关键）
        # ========================
        rospy.loginfo("Waiting for publishers to connect...")
        rospy.sleep(1.0)

        rospy.loginfo("Sending takeoff signal...")
        self.pub_takeoff.publish(Empty())

        rospy.sleep(1.0)

        rospy.loginfo("Switching to velocity/acceleration control mode...")
        self.pub_vel_mode.publish(Bool(data=True))

        rospy.loginfo("BC control is now active.")

    def odom_cb(self, msg):
        self.last_odom = msg

    def _build_state_from_odom(self, odom):
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
        rospy.loginfo("BCPolicyNode running. Publishing cmd_acc...")
        while not rospy.is_shutdown():
            if self.last_odom is not None:
                state = self._build_state_from_odom(self.last_odom)
                with torch.no_grad():
                    action = self.model(state).squeeze(0).cpu().numpy()

                cmd = Vector3(
                    x=float(action[0]),
                    y=float(action[1]),
                    z=float(action[2]),
                )

                self.pub_cmd.publish(cmd)

            self.rate.sleep()


def main():
    rospy.init_node('bc_policy_node')
    node = BCPolicyNode()
    node.run()


if __name__ == '__main__':
    main()
