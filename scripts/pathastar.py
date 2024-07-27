#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf
import heapq

def create_path(occupancy_grid, resolution, origin):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    start_world = (91.858, 124.974)
    goal_world = (95.751, 120.128)

    start = (int((start_world[0] - origin.x) / resolution), int((start_world[1] - origin.y) / resolution))
    goal = (int((goal_world[0] - origin.x) / resolution), int((goal_world[1] - origin.y) / resolution))

    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    occupancy_data = np.array(occupancy_grid.data).reshape((height, width))

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        neighbors = [
            (current[0] + 1, current[1]), (current[0] - 1, current[1]),
            (current[0], current[1] + 1), (current[0], current[1] - 1)
        ]
        for neighbor in neighbors:
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                if occupancy_data[neighbor[1], neighbor[0]] == 0:  # Free space
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return [] 

def path_to_ros_path(path, resolution, origin):
    ros_path = Path()
    ros_path.header.frame_id = "map"
    ros_path.header.stamp = rospy.Time.now()

    for (x, y) in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = origin.x + x
        pose.pose.position.y = origin.y + y
        pose.pose.position.z = origin.z
        pose.pose.orientation = Quaternion(0, 0, 0, 1)

        ros_path.poses.append(pose)

    return ros_path

def occupancy_grid_callback(msg):
    global latest_occupancy_grid
    latest_occupancy_grid = msg

def main():
    global latest_occupancy_grid
    latest_occupancy_grid = None

    rospy.init_node('path_planner', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, occupancy_grid_callback)
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if latest_occupancy_grid is not None:
            resolution = latest_occupancy_grid.info.resolution
            origin = latest_occupancy_grid.info.origin.position
            path = create_path(latest_occupancy_grid, resolution, origin)
            if path:
                ros_path = path_to_ros_path(path, resolution, origin)
                path_pub.publish(ros_path)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
