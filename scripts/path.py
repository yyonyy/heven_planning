#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf

def create_path():

    start = (63.713, 147.021)  # start point in grid coordinates
    goal = (83.748, 109.983)   # goal point in grid coordinates

    # path planning algorithm
    # straight line 
    path = []
    steps = int(max(abs(goal[0] - start[0]), abs(goal[1] - start[1])))
    for i in range(steps + 1):
        x = int(start[0] + i * (goal[0] - start[0]) / steps)
        y = int(start[1] + i * (goal[1] - start[1]) / steps)
        path.append((x, y))

    return path

def path_to_ros_path(path, resolution, origin):
    ros_path = Path()
    ros_path.header.frame_id = "map"
    ros_path.header.stamp = rospy.Time.now()

    for (x, y) in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        # Convert grid coordinates to world coordinates
        # pose.pose.position.x = origin.x + x * resolution
        # pose.pose.position.y = origin.y + y * resolution
        # pose.pose.position.z = origin.z
        pose.pose.position.x =  x
        pose.pose.position.y =  y 
        pose.pose.position.z = origin.z
        pose.pose.orientation = Quaternion(0, 0, 0, 1)

        ros_path.poses.append(pose)

    return ros_path

def main():
    rospy.init_node('path_planner', anonymous=True)
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rate = rospy.Rate(1)

    resolution = 0.05
    origin = Point(0, 0, 0) 

    while not rospy.is_shutdown():
        path = create_path()
        print(path)
        ros_path = path_to_ros_path(path, resolution, origin)
        # print(ros_path)
        path_pub.publish(ros_path)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
