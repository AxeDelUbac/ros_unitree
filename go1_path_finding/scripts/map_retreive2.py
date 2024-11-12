#!/usr/bin/env python3
from numpy import rate
import rospy
import math
import actionlib
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PathFindingAlgorithm:
    def __init__(self):
        #subscription and publication
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal_position)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.start_end_path_pub = rospy.Publisher("/start_end_path", Path, queue_size=10)
        # self.global_plan_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        
        # Publisher pour afficher la trajectoire enregistrée
        self.actual_path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)

        # Liste pour enregistrer les poses de la trajectoire
        self.actual_trajectory = []

        # Données de la carte, position du robot et position de l'objectif
        self.map_data = None
        self.robot_position = None

        # Variables de trajet
        self.planned_trajectory = []  # Stocker la trajectoire sinusoïdale
        self.current_target_index = 0  # Indice pour suivre le point actuel

        self.goal_position = None
        self.initial_position = None

        # Threshold for reaching goal (in meters)
        self.goal_reached_threshold = 0.2

        # Initialisation de last_publish_time pour gérer la fréquence de publication
        self.last_publish_time = rospy.Time.now()

        # Action client pour envoyer des objectifs à move_base
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()

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

    def goal_reached_callback(self, status, result):
        """Callback pour gérer l'atteinte de chaque objectif."""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Point {self.current_target_index} atteint.")
            self.current_target_index += 1
            # Annuler explicitement l'objectif pour éviter les conflits d'état
            self.move_base_client.cancel_goal()
            self.move_base_client.wait_for_result()  # Assure l'attente de la fin
            # Envoyer le prochain objectif après l'annulation
            self.send_next_goal()
        else:
            rospy.logwarn("L'objectif n'a pas été atteint. Passage au point suivant.")

            # Annuler l'objectif pour s'assurer qu'il est réinitialisé
            self.move_base_client.cancel_goal()
            self.move_base_client.wait_for_result()

            self.current_target_index += 1
            self.send_next_goal()

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

        # Enregistre la pose actuelle dans la trajectoire
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.robot_position[0]
        pose.pose.position.y = self.robot_position[1]
        #pose.pose.orientation = msg.pose.pose.orientation
        # Correction de l'orientation pour éviter le quaternion invalide
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.actual_trajectory.append(pose)

        # Information de débogage
        self.publish_actual_path()
        rospy.loginfo(f"Nouvelle pose ajoutée : {self.robot_position}")

        # Si c'est la première mise à jour de la position, on la garde comme position initiale
        if self.initial_position is None:
            self.initial_position = self.robot_position
        
        # If a goal is defined, check if we've reached it
        if self.goal_position and self.has_reached_goal():
            # self.update_initial_position()  # Update the initial position with the new position
            self.publish_initial_path()

    def update_goal_position(self, msg):
        """Mise à jour de la position de l'objectif."""
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Position de l'objectif : {self.goal_position}")

         # Réinitialiser la trajectoire pour le nouveau trajet
        self.actual_trajectory = []

       # Si la position initiale du robot est connue, trace la ligne entre la position initiale et l'objectif
        if self.initial_position:
            self.publish_initial_path() 
        
        # Si la position du robot est connue, trace la ligne
        # if self.robot_position:
            # self.publish_path()
            # self.publish_curved_path()

        # Génère la trajectoire sinusoïdale entre la position actuelle et l'objectif
        if self.robot_position:
            self.publish_curved_path()
            self.send_next_goal()

    def has_reached_goal(self):
        """Check if the robot has reached its goal position."""
        if self.robot_position and self.goal_position:
            distance = math.sqrt(
                (self.goal_position[0] - self.robot_position[0]) ** 2 +
                (self.goal_position[1] - self.robot_position[1]) ** 2
            )
            return distance < self.goal_reached_threshold
            # Réinitialiser la trajectoire pour le nouveau trajet   
        return False

    def publish_distance(self):
        """Calcule et publie la distance entre la position du robot et l'objectif."""
        if self.robot_position is None or self.goal_position is None:
            return

        # Calcul de la distance entre la position du robot et l'objectif
        distance = math.sqrt((self.goal_position[0] - self.robot_position[0]) ** 2 +
                            (self.goal_position[1] - self.robot_position[1]) ** 2)

        # Publie la distance
        self.distance_pub.publish(distance)
        rospy.loginfo(f"Distance jusqu'à l'objectif : {distance:.2f} m")


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
        # self.path_pub = rospy.Publisher("/planned_path", path_msg, queue_size=10)
        rospy.loginfo("Ligne publiée entre le robot et l'objectif")

    def publish_initial_path(self):
        """Trace une ligne entre la position initiale du robot et l'objectif."""
        initial_path_msg = Path()
        initial_path_msg.header.frame_id = "map"
        initial_path_msg.header.stamp = rospy.Time.now()

        # Pose de départ (position initiale du robot)
        start_pose = PoseStamped()
        start_pose.pose.position.x = self.initial_position[0]
        start_pose.pose.position.y = self.initial_position[1]
        start_pose.pose.orientation.w = 1.0
        initial_path_msg.poses.append(start_pose)

        # Pose de fin (position de l'objectif)
        end_pose = PoseStamped()
        end_pose.pose.position.x = self.goal_position[0]
        end_pose.pose.position.y = self.goal_position[1]
        end_pose.pose.orientation.w = 1.0
        initial_path_msg.poses.append(end_pose)

        # Publie le chemin initial
        self.start_end_path_pub.publish(initial_path_msg)
        rospy.loginfo("Ligne initiale publiée entre la position de départ et l'objectif")

        # Publier sur le topic du global plan pour le move_base
        self.global_plan_pub.publish(initial_path_msg)
        rospy.loginfo("Ligne initiale publiée sur /move_base/TrajectoryPlannerROS/global_plan")

    def publish_curved_path(self):
        """Trace une courbe entre la position du robot et l'objectif."""
        if not self.robot_position or not self.goal_position:
            rospy.logwarn("Position du robot ou de l'objectif non définie, impossible de générer le chemin.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Réinitialiser la trajectoire existante
        self.planned_trajectory = []

        # Génération des poses avec une courbe sinusoïdale entre le point de départ et le point d'arrivée
        num_points = 50
        x_start, y_start = self.robot_position
        x_end, y_end = self.goal_position

        for i in range(num_points + 1):
            t = i / num_points  # Paramètre t variant de 0 à 1
            x = (1 - t) * x_start + t * x_end  # Interpolation linéaire sur x
            y = (1 - t) * y_start + t * y_end + 1.5 * math.sin(2 * math.pi * t)  # Ajout d'une composante sinusoïdale sur y

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.planned_trajectory.append(pose)

        # Publie le chemin courbe
        self.path_pub.publish(path_msg)
        rospy.loginfo("Trajectoire courbe publiée sur /planned_path")

    def publish_actual_path(self):
        """Publie le chemin parcouru par le robot pour visualisation dans Rviz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.actual_trajectory

        # Publie le chemin parcouru sur /actual_path
        self.actual_path_pub.publish(path_msg)
        rospy.loginfo("Chemin parcouru publié sur /actual_path")

    def send_next_goal(self):
        """Envoie le prochain point de la sinusoïde à move_base comme objectif."""
        if self.current_target_index < len(self.planned_trajectory):
            target_pose = self.planned_trajectory[self.current_target_index]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = target_pose.pose

            rospy.loginfo(f"Envoi du point {self.current_target_index} comme objectif à move_base : {goal.target_pose.pose.position}")
            self.move_base_client.send_goal(goal, done_cb=self.goal_reached_callback)

            # Ajouter une pause pour laisser le temps à move_base de traiter le nouvel objectif
            rospy.sleep(0.1)
        else:
            rospy.loginfo("Tous les points de la trajectoire sinusoïdale ont été atteints.")

    def run(self):
        # rospy.spin()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            
            #self.publish_custom_path()  # Publie le chemin sinusoïdal
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_finding_algorithm2")

    #path_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        # Attendez un moment que le publisher se connecte
    rospy.sleep(1)
    

    # publish_line(start_point, end_point)
    
    rospy.loginfo("Test Started")

    pfa = PathFindingAlgorithm()

    pfa.run()