#!/usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

def point_cloud_to_ros(points):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # Ensure the frame_id is "map"
    pc2 = point_cloud2.create_cloud_xyz32(header, points)
    return pc2

def pgm_to_occupancy_grid(pgm_file, resolution):
    # Load PGM file
    img = Image.open(pgm_file)
    img = img.convert('L')
    img = np.array(img)
    
    # Convert image to binary occupancy grid
    occupancy_grid = np.where(img > 128, 0, 100)  # Assuming >128 is free, <=128 is occupied

    # Create OccupancyGrid message
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"  # Ensure the frame_id is "map"
    grid_msg.info.resolution = resolution
    grid_msg.info.width = img.shape[1]
    grid_msg.info.height = img.shape[0]
    grid_msg.info.origin.position.x = 0  # Adjust these values if necessary
    grid_msg.info.origin.position.y = 0
    grid_msg.info.origin.position.z = 0
    grid_msg.info.origin.orientation.w = 1.0

    flat_grid = occupancy_grid.flatten(order='C')
    grid_msg.data = list(np.int8(flat_grid))
    return grid_msg

def main():
    rospy.init_node('pgm_pcd_publisher', anonymous=True)
    pcl_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

    # Load the point cloud
    pcd = o3d.io.read_point_cloud("/home/heven/catkin_ws/src/grid_map_pcl/data/GlobalMap.pcd")
    points = np.asarray(pcd.points)

    # Load and convert PGM to OccupancyGrid
    pgm_file = "/home/heven/catkin_ws/src/grid_map_pcl/occupancy_grid.pgm"
    resolution = 0.05  # Set the appropriate resolution
    grid_msg = pgm_to_occupancy_grid(pgm_file, resolution)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Convert point cloud to ROS PointCloud2 message and publish
        pcl_msg = point_cloud_to_ros(points)
        pcl_pub.publish(pcl_msg)

        # Publish the occupancy grid
        grid_pub.publish(grid_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
