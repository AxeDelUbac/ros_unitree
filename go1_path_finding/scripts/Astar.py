#!/usr/bin/env python3

import rospy
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class AStarPlanner:
    def __init__(self):
        # Subscribe to the map
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher("/astar_path", Path, queue_size=10)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        self.map_data = None
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.map_data = msg
        rospy.loginfo("Map received.")

    def start_callback(self, msg):
        self.start = (int(msg.pose.pose.position.x), int(msg.pose.pose.position.y))
        rospy.loginfo(f"Start position set to: {self.start}")

    def goal_callback(self, msg):
        self.goal = (int(msg.pose.position.x), int(msg.pose.position.y))
        rospy.loginfo(f"Goal position set to: {self.goal}")
        if self.map_data and self.start and self.goal:
            self.plan_path()

    def heuristic(self, a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal, map_data):
        width = map_data.info.width
        height = map_data.info.height
        map_array = list(map_data.data)

        def get_neighbors(pos):
            neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4 directions (N, E, S, W)
            result = []
            for direction in neighbors:
                neighbor = (pos[0] + direction[0], pos[1] + direction[1])
                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                    index = neighbor[1] * width + neighbor[0]
                    if map_array[index] == 0:  # Free space
                        result.append(neighbor)
            return result

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # No path found

    def plan_path(self):
        rospy.loginfo("Planning path using A*...")
        path = self.astar(self.start, self.goal, self.map_data)

        if path:
            rospy.loginfo("Path found!")
            path_msg = Path()
            path_msg.header.frame_id = "map"
            for position in path:
                pose = PoseStamped()
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
        else:
            rospy.logwarn("No path found!")

if __name__ == '__main__':
    rospy.init_node('astar_planner')
    rospy.loginfo("Node astar_planner has been started")
    AStarPlanner()
    rospy.spin()