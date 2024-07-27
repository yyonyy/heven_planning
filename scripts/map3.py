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

def grid_map_to_ros(grid, resolution, origin):
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"
    grid_msg.info.resolution = resolution
    grid_msg.info.width = grid.shape[1]
    grid_msg.info.height = grid.shape[0]
    grid_msg.info.origin.position.x = origin[0]
    grid_msg.info.origin.position.y = origin[1]
    grid_msg.info.origin.position.z = 0
    grid_msg.info.origin.orientation.w = 1.0

    flat_grid = grid.flatten(order='C')
    grid_msg.data = list(np.int8(flat_grid))
    return grid_msg

def main():
    rospy.init_node('grid_map_pcl_node', anonymous=True)
    pcl_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

    # Load the point cloud
    pcd = o3d.io.read_point_cloud("/home/heven/catkin_ws/src/grid_map_pcl/data/GlobalMap.pcd")
    points = np.asarray(pcd.points)

    # Projecting onto the XY plane
    points_2d = points[:, :2]

    # Define grid resolution
    resolution = 0.05  # grid cell size

    # Calculate the minimum and maximum coordinates
    min_coords = points_2d.min(axis=0)
    max_coords = points_2d.max(axis=0)

    # Calculate the size of the grid
    grid_size = np.ceil((max_coords - min_coords) / resolution).astype(int)

    # Initialize the occupancy grid
    occupancy_grid = np.zeros(grid_size, dtype=np.uint8)

    # Calculate the grid indices for each point
    grid_indices = ((points_2d - min_coords) / resolution).astype(int)

    # Mark the occupied cells
    for idx in grid_indices:
        occupancy_grid[idx[0], idx[1]] = 255  # Mark as occupied

    # Create an image from the occupancy grid
    img = Image.fromarray(occupancy_grid)

    # Save as PGM
    img.save("/home/heven/catkin_ws/src/map/occupancy_grid.pgm")

    # Convert occupancy grid to ROS OccupancyGrid message
    origin = min_coords
    grid_msg = grid_map_to_ros(occupancy_grid, resolution, origin)

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
