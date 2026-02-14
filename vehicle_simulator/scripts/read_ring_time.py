#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def callback(msg):
    # 检查字段
    field_names = [f.name for f in msg.fields]
    if "ring" not in field_names or "time" not in field_names:
        rospy.logwarn("PointCloud does not contain 'ring' or 'time' field!")
        return

    # 读取前10个点
    count = 0
    for point in pc2.read_points(msg, field_names=("x", "y", "z", "ring", "time"), skip_nans=True):
        x, y, z, ring, time = point
        rospy.loginfo("Point #%d: ring=%d, time=%.6f" % (count, int(ring), time))
        count += 1
        if count >= 10:
            break

def main():
    rospy.init_node("read_ring_time_node", anonymous=True)
    rospy.Subscriber("/vehicle0/sensor_scan", PointCloud2, callback, queue_size=1)
    rospy.loginfo("Listening to /vehicle0/velodyne_points ...")
    rospy.spin()

if __name__ == "__main__":
    main()

