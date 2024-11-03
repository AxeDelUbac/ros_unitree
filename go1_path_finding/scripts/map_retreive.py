#!/usr/bin/env python3
from numpy import rate
import rospy
import math
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped


class PathFindingAlgorithm:
    def __init__(self):
        #subscription and publication
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal_position)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.goal_path_pub = rospy.Publisher("/goal_path", Path, queue_size=10)

        # Données de la carte, position du robot et position de l'objectif
        self.map_data = None
        self.robot_position = None
        self.goal_position = None

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

    # def publish_custom_path(self):
    #     path = Path()
    #     path.header.frame_id = "odom"
    #     path.header.stamp = rospy.Time.now()

    #     # Génération des poses avec une courbe sinusoïdale
    #     for i in range(50):
    #         pose = PoseStamped()
    #         pose.header.frame_id = "odom"
    #         pose.pose.position.x = i * 0.1
    #         pose.pose.position.y = math.sin(i * 0.2)
    #         pose.pose.position.z = 0
    #         pose.pose.orientation.w = 1.0
    #         path.poses.append(pose)

    #     # Publie le chemin sinusoïdal
    #     self.path_pub.publish(path)

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
        if self.robot_position:
            self.publish_path()

    def publish_path(self):
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

    def run(self):
        # rospy.spin()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            #self.publish_custom_path()  # Publie le chemin sinusoïdal
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_finding_algorithm")

    #path_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        # Attendez un moment que le publisher se connecte
    rospy.sleep(1)
    

    # publish_line(start_point, end_point)
    
    rospy.loginfo("Test Started")

    pfa = PathFindingAlgorithm()

    pfa.run()