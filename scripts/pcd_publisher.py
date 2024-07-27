#!/usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
from PIL import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import cv2

def point_cloud_to_ros(points):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    pc2 = point_cloud2.create_cloud_xyz32(header, points)
    return pc2

def main():
    rospy.init_node('pcd_publisher', anonymous=True)
    pcl_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

    # Load the point cloud
    pcd = o3d.io.read_point_cloud("/home/heven/catkin_ws/src/map/maps/GlobalMap.pcd")
    points = np.asarray(pcd.points)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pcl_msg = point_cloud_to_ros(points)
        pcl_pub.publish(pcl_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
