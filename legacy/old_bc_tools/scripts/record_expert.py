#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TwistStamped


class ExpertRecorder(object):
    def __init__(self):
        default_dir = os.path.expanduser('~/uav_bc_training/data')
        self.output_dir = rospy.get_param('~output_dir', default_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        rospy.loginfo("ExpertRecorder output_dir = %s", self.output_dir)

        self.timestamps = []
        self.positions = []
        self.orientations = []
        self.lin_vels = []
        self.ang_vels = []
        self.actions = []
        self.action_source = []

        self.last_odom = None

        # odom / odom_raw 谁有数据用谁
        self.sub_odom = rospy.Subscriber(
            '/CERLAB/quadcopter/odom',
            Odometry,
            self.odom_cb,
            queue_size=50
        )
        self.sub_odom_raw = rospy.Subscriber(
            '/CERLAB/quadcopter/odom_raw',
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        # 动作 1：加速度命令（Vector3）
        self.sub_cmd_acc = rospy.Subscriber(
            '/CERLAB/quadcopter/cmd_acc',
            Vector3,
            self.cmd_acc_cb,
            queue_size=50
        )

        # 动作 2：速度命令（TwistStamped）
        self.sub_cmd_vel = rospy.Subscriber(
            '/CERLAB/quadcopter/cmd_vel',
            TwistStamped,
            self.cmd_vel_cb,
            queue_size=50
        )

        rospy.loginfo("ExpertRecorder is ready. Start flying and I will record :)")

    def odom_cb(self, msg):
        self.last_odom = msg

    def _record_sample(self, action_vec, source_name):
        if self.last_odom is None:
            return

        stamp = rospy.Time.now().to_sec()
        odom = self.last_odom

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular

        position = [p.x, p.y, p.z]
        orientation = [q.x, q.y, q.z, q.w]
        lin_vel = [lv.x, lv.y, lv.z]
        ang_vel = [av.x, av.y, av.z]

        self.timestamps.append(stamp)
        self.positions.append(position)
        self.orientations.append(orientation)
        self.lin_vels.append(lin_vel)
        self.ang_vels.append(ang_vel)
        self.actions.append(list(action_vec))
        self.action_source.append(source_name)

        if len(self.timestamps) % 50 == 0:
            rospy.loginfo("Recorded %d samples (last source: %s)",
                          len(self.timestamps), source_name)

    def cmd_acc_cb(self, msg):
        # Vector3: (ax, ay, az)
        action_vec = (msg.x, msg.y, msg.z)
        self._record_sample(action_vec, "cmd_acc")

    def cmd_vel_cb(self, msg):
        # TwistStamped: 取 msg.twist.linear
        v = msg.twist.linear
        action_vec = (v.x, v.y, v.z)
        self._record_sample(action_vec, "cmd_vel")

    def save_to_file(self):
        if len(self.timestamps) == 0:
            rospy.logwarn("No expert data recorded, skip saving.")
            return

        data = {
            't': np.array(self.timestamps, dtype=np.float64),
            'position': np.array(self.positions, dtype=np.float32),
            'orientation': np.array(self.orientations, dtype=np.float32),
            'lin_vel': np.array(self.lin_vels, dtype=np.float32),
            'ang_vel': np.array(self.ang_vels, dtype=np.float32),
            'action': np.array(self.actions, dtype=np.float32),
            'action_source': np.array(self.action_source),
        }

        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'expert_{now_str}.npz')

        np.savez_compressed(filename, **data)
        rospy.loginfo("Saved expert data to %s", filename)
        rospy.loginfo("Total samples: %d", len(self.timestamps))


def main():
    rospy.init_node('expert_recorder')

    recorder = ExpertRecorder()
    rospy.on_shutdown(recorder.save_to_file)

    rospy.spin()


if __name__ == '__main__':
    main()
