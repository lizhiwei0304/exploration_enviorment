#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Author: lee lizw_0304@163.com
Date: 2025-07-08 11:23:26
LastEditors: lee lizw_0304@163.com
LastEditTime: 2025-07-08 11:30:00
FilePath: /AUTO_DEVELOPMENT/vehicle_simulator/scripts/transform_cloud_node.py
Description: 订阅车辆位姿和原始点云，将点云转换到 map 坐标系下发布
"""

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry  # ✅ 正确的消息类型
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloudTransformer:
    def __init__(self):
        rospy.init_node("pointcloud_transformer")

        self.robot_namespace = rospy.get_param("~robot_namespace", "vehicle0")
        self.pose_topic = "/" + self.robot_namespace + "/state_estimation_noisy"
        self.scan_topic = "/" + self.robot_namespace + "/sensor_scan"

        self.latest_pose = None
        self.tf_listener = tf.TransformListener()

        # ✅ 订阅 nav_msgs/Odometry
        rospy.Subscriber(self.pose_topic, Odometry, self.pose_callback)
        rospy.Subscriber(self.scan_topic, PointCloud2, self.scan_callback)

        self.pub = rospy.Publisher(
            "/" + self.robot_namespace + "/transformed_cloud", PointCloud2, queue_size=1
        )
        rospy.loginfo(
            "PointCloud Transformer Initialized for [%s]" % self.robot_namespace
        )

    def pose_callback(self, msg):
        # ✅ 提取 pose.pose，而不是 PoseStamped
        self.latest_pose = msg.pose.pose

    def scan_callback(self, cloud_msg):
        if self.latest_pose is None:
            rospy.logwarn_throttle(5.0, "No pose received yet.")
            return

        # 提取位姿
        position = self.latest_pose.position
        orientation = self.latest_pose.orientation
        trans = (position.x, position.y, position.z)
        rot = (orientation.x, orientation.y, orientation.z, orientation.w)

        # 构造 4x4 的变换矩阵
        T = self.get_transform_matrix(trans, rot)

        # 点云转换
        points = list(
            pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        )
        transformed_points = []
        for p in points:
            pt = np.array([p[0], p[1], p[2], 1.0])
            pt_map = T.dot(pt)
            transformed_points.append((pt_map[0], pt_map[1], pt_map[2]))

        # 构造新的 PointCloud2 消息
        header = Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = "map"  # 可根据需要改为 odom 或 base_link

        cloud_out = pc2.create_cloud_xyz32(header, transformed_points)
        self.pub.publish(cloud_out)

    def get_transform_matrix(self, translation, quaternion):
        tf_matrix = tf.transformations.quaternion_matrix(quaternion)
        tf_matrix[0:3, 3] = translation
        return tf_matrix


if __name__ == "__main__":
    try:
        transformer = PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
