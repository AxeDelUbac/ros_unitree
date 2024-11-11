#!/usr/bin/env python3
from numpy import rate
import rospy
import math
import heapq
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped


class PathFindingAlgorithm:
    def __init__(self):
        #subscription and publication
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal_position)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.global_plan_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)

        # Données de la carte, position du robot et position de l'objectif
        self.map_data = None
        self.robot_position = None
        self.goal_position = None

        # To track previous state
        self.previous_robot_position = None
        self.previous_goal_position = None

    def map_callback(self, msg):
        self.map_data = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Convertir la grille d'occupation en tableau 2D
        map_array = list(msg.data)  # Convertir la liste des données en tableau
        map_grid = [map_array[i * width:(i + 1) * width] for i in range(height)]
        
       
        rospy.loginfo(f"Carte de {width}x{height} reçue avec une résolution de {resolution} mètre/cellule.")

    def update_robot_position(self, msg):
        """Mise à jour de la position actuelle du robot."""
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Position actuelle du robot : {self.robot_position}")
        
        # Si un goal est défini, trace la ligne
        if self.goal_position:
            self.publish_path()

    def update_goal_position(self, msg):
        """Mise à jour de la position de l'objectif."""
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Position de l'objectif : {self.goal_position}")
        
        # Si la position du robot est connue, trace la ligne
        # if self.robot_position:
        #     self.publish_path()

    def create_grid_from_map(self):
        """Convert the map into a 2D grid usable for A*"""
        if self.map_data is None:
            return None
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = list(self.map_data.data)
        grid = [map_array[i * width:(i + 1) * width] for i in range(height)]
        return grid
    
    def heuristic(self, start, goal):
        """Heuristique pour A* (distance euclidienne)"""
        return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)

    def get_neighbors(self, position, grid):
        """Retourne les voisins accessibles d'une position donnée"""
        x, y = position
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4 directions (ajoutez des diagonales si nécessaire)
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid):
                if grid[ny][nx] == 0:  # 0 signifie libre
                    neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruction du chemin après A*"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star(self, start, goal, grid):
        """Implémentation de l'algorithme A*"""
        open_set = []
        heapq.heappush(open_set, (0, start))  # (f, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # Si on est arrivé au but
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current, grid):
                tentative_g_score = g_score[current] + 1  # Chaque mouvement coûte 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # Aucun chemin trouvé

    def startEndLineDrawing(self):
        """Trace une ligne entre la position du robot et l'objectif."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Pose de départ (position du robot)
        start_pose = PoseStamped()
        start_pose.pose.position.x = self.robot_position[0]
        start_pose.pose.position.y = self.robot_position[1]
        start_pose.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose)

        # Pose de fin (position de l'objectif)
        end_pose = PoseStamped()
        end_pose.pose.position.x = self.goal_position[0]
        end_pose.pose.position.y = self.goal_position[1]
        end_pose.pose.orientation.w = 1.0
        path_msg.poses.append(end_pose)

        # Publie le chemin
        self.path_pub.publish(path_msg)
        self.path_pub = rospy.Publisher("/planned_path", path_msg, queue_size=10)
        rospy.loginfo("Ligne publiée entre le robot et l'objectif")

    def publish_path(self):
        """Calculate and publish the A* path between the robot's position and the goal"""
        if not self.robot_position or not self.goal_position or not self.map_data:
            return

        grid = self.create_grid_from_map()
        if grid is None:
            rospy.logwarn("La carte n'est pas encore prête.")
            return

        start = (int(self.robot_position[0]), int(self.robot_position[1]))
        goal = (int(self.goal_position[0]), int(self.goal_position[1]))

        # Vérifier si les positions de départ et de but sont valides
        if grid[start[1]][start[0]] != 0:
            rospy.logwarn("La position de départ est invalide (occupée ou inconnue).")
            return

        if grid[goal[1]][goal[0]] != 0:
            rospy.logwarn("La position de l'objectif est invalide (occupée ou inconnue).")
            return

        path = self.a_star(start, goal, grid)
        if path:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = rospy.Time.now()

            for (x, y) in path:
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            # Publish the A* path to both topics
            self.path_pub.publish(path_msg)
            rospy.loginfo("Chemin A* publié sur /planned_path")
            self.global_plan_pub.publish(path_msg)
            rospy.loginfo("Chemin A* publié sur /move_base/TrajectoryPlannerROS/global_plan")
        else:
            rospy.logwarn("Aucun chemin trouvé par A*")

    def run(self):
         rospy.spin()
        # rate = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     # Réévaluer et publier le chemin en continu si des données sont disponibles
        #     if self.robot_position and self.goal_position and self.map_data:
        #         self.publish_path()
        #     rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_finding_algorithm")

    #path_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        # Attendez un moment que le publisher se connecte
    rospy.sleep(1)
    
    
    rospy.loginfo("Test Started")

    pfa = PathFindingAlgorithm()

    pfa.run()