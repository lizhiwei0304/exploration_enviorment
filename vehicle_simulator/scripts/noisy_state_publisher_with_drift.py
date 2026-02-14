#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_from_euler


class DistanceBasedDriftPublisher:
    def __init__(self):
        rospy.init_node("distance_based_drift_publisher")

        # 获取命名空间
        self.robot_namespace = rospy.get_param("~robot_namespace", "vehicle0")
        rospy.loginfo("Robot Namespace: %s" % self.robot_namespace)

        # ----------------------------
        # 噪声参数
        # ----------------------------
        self.position_noise_std = rospy.get_param(
            "~position_noise_std", 0.2
        )  # XY白噪声（米）

        self.orientation_noise_std_deg = rospy.get_param(
            "~orientation_noise_std_deg", 1.0
        )  # 姿态白噪声（度）

        self.position_drift_per_meter = rospy.get_param(
            "~position_drift_per_meter", 0.2
        )  # XY漂移（米/米）

        self.orientation_drift_per_meter_deg = rospy.get_param(
            "~orientation_drift_per_meter_deg", 0.5
        )  # 姿态漂移（度/米）

        # 状态
        self.position_drift = np.zeros(3)  # x, y, z（z不会变化）
        self.orientation_drift_rpy = np.zeros(3)  # roll, pitch, yaw

        self.last_position = None
        self.total_distance = 0.0

        # ----------------------------
        # ROS 通信
        # ----------------------------
        input_topic = "/" + self.robot_namespace + "/state_estimation_at_scan"
        output_topic = "/" + self.robot_namespace + "/state_estimation_noisy"

        self.sub = rospy.Subscriber(input_topic, Odometry, self.callback)
        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=10)

        rospy.loginfo("Subscribed to: %s" % input_topic)
        rospy.loginfo("Publishing noisy state to: %s" % output_topic)

        rospy.spin()

    def callback(self, msg):
        # 当前帧位置
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        current_position = np.array([px, py, pz])

        # 计算增量距离（只考虑XY，不计Z）
        if self.last_position is None:
            delta_distance = 0.0
        else:
            delta_position = current_position - self.last_position
            delta_distance = np.linalg.norm(delta_position[:2])  # 只用XY
        self.total_distance += delta_distance
        self.last_position = current_position

        # ----------------------------
        # 更新漂移状态（XY）
        # ----------------------------
        position_drift_step = np.random.normal(
            0, self.position_drift_per_meter * delta_distance, size=2
        )  # XY
        self.position_drift[0] += position_drift_step[0]
        self.position_drift[1] += position_drift_step[1]
        # Z 不变

        orientation_drift_step = np.random.normal(
            0, self.orientation_drift_per_meter_deg * delta_distance, size=3
        )
        self.orientation_drift_rpy += np.deg2rad(orientation_drift_step)

        # ----------------------------
        # 构造带噪声的消息
        # ----------------------------
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id

        # 位置 = 原始 + XY白噪声 + XY漂移，Z不加噪声不漂移
        noisy_msg.pose.pose.position.x = (
            px + np.random.normal(0, self.position_noise_std) + self.position_drift[0]
        )
        noisy_msg.pose.pose.position.y = (
            py + np.random.normal(0, self.position_noise_std) + self.position_drift[1]
        )
        noisy_msg.pose.pose.position.z = pz  # 保持原始Z

        # 姿态 = 原始 * (白噪声) * (漂移)
        ori = msg.pose.pose.orientation
        q_orig = [ori.x, ori.y, ori.z, ori.w]

        rpy_noise = np.deg2rad(
            np.random.normal(0, self.orientation_noise_std_deg, size=3)
        )
        q_noise = quaternion_from_euler(*rpy_noise)

        q_drift = quaternion_from_euler(*self.orientation_drift_rpy)

        q_new = quaternion_multiply(quaternion_multiply(q_orig, q_noise), q_drift)

        noisy_msg.pose.pose.orientation.x = q_new[0]
        noisy_msg.pose.pose.orientation.y = q_new[1]
        noisy_msg.pose.pose.orientation.z = q_new[2]
        noisy_msg.pose.pose.orientation.w = q_new[3]

        # 保持速度
        noisy_msg.twist = msg.twist

        # 发布
        self.pub.publish(noisy_msg)

        rospy.loginfo_throttle(
            5,
            "[%s] Total Distance: %.2f m, Drift XY: [%.2f, %.2f]"
            % (
                self.robot_namespace,
                self.total_distance,
                self.position_drift[0],
                self.position_drift[1],
            ),
        )


if __name__ == "__main__":
    try:
        DistanceBasedDriftPublisher()
    except rospy.ROSInterruptException:
        pass
