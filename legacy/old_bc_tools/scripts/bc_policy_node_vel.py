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

        # 你训练脚本保存的位置（默认与你 train_bc_vel.py 一致）
        model_path = os.path.expanduser("~/uav_bc_training/models/bc_policy_vel.pt")
        stats_path = os.path.expanduser("~/uav_bc_training/models/bc_vel_stats.npz")

        if not os.path.exists(model_path):
            rospy.logerr("Model not found: %s", model_path)
            raise RuntimeError("Model not found")
        if not os.path.exists(stats_path):
            rospy.logerr("Stats not found: %s", stats_path)
            raise RuntimeError("Stats not found")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo("Using device: %s", self.device)

        # 模型
        self.model = BCPolicy().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        # ✅ 载入正规化统计量（训练时保存的）
        stats = np.load(stats_path)
        self.state_mean = torch.from_numpy(stats["state_mean"]).float().to(self.device)  # (13,)
        self.state_std  = torch.from_numpy(stats["state_std"]).float().to(self.device)   # (13,)
        self.action_mean = stats["action_mean"].astype(np.float32)  # (3,)
        self.action_std  = stats["action_std"].astype(np.float32)   # (3,)

        rospy.loginfo("Loaded stats: %s", stats_path)

        # （可选）速度上限，单位 m/s。建议设成你专家数据的合理范围，比如 1.0~2.0
        self.v_max = rospy.get_param("~v_max", 2.0)

        self.last_odom = None
        rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self.odom_cb, queue_size=1)

        # 发布 TwistStamped cmd_vel
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

        # (1, 13)
        return torch.from_numpy(state).unsqueeze(0).to(self.device)

    def run(self):
        rospy.loginfo("Publishing TwistStamped cmd_vel from BC (normalized)...")
        while not rospy.is_shutdown():
            if self.last_odom is not None:
                s = self.build_state(self.last_odom)  # (1, 13)

                # ✅ 推理时同样做 state 标准化
                s_norm = (s - self.state_mean) / self.state_std

                with torch.no_grad():
                    a_norm = self.model(s_norm).squeeze(0).cpu().numpy()  # (3,)

                # ✅ 反标准化回真实速度单位（m/s）
                v = a_norm * self.action_std + self.action_mean  # (3,)

                # ✅ （可选）限幅，避免突然爆炸速度
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


def main():
    rospy.init_node("bc_policy_node_vel")
    node = BCPolicyVelNode()
    node.run()


if __name__ == "__main__":
    main()
