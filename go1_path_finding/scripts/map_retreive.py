#!/usr/bin/env python3
import rospy
import math
import actionlib
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PathFindingAlgorithm:
    def __init__(self):
        # Initialisation des abonnements et publications
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal_position)

        self.global_plan_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.actual_path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)
        self.reached_points_pub = rospy.Publisher("/reached_points", PoseStamped, queue_size=10)

        # Liste pour enregistrer les poses de la trajectoire
        self.actual_trajectory = []

        self.reached_points = []

        # Données de la carte, position du robot et position de l'objectif
        self.map_data = None
        self.robot_position = None
        self.goal_position = None
        self.initial_position = None

        # Variables de trajet
        self.planned_trajectory = []  # Stocker la trajectoire sinusoïdale
        self.current_target_index = 0  # Indice pour suivre le point actuel

        # Action client pour envoyer des objectifs à move_base
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()

    def map_callback(self, msg):
        rospy.loginfo(f"Carte de {msg.info.width}x{msg.info.height} reçue avec une résolution de {msg.info.resolution} mètre/cellule.")
        self.map_data = msg

    def update_robot_position(self, msg):
        """Mise à jour de la position actuelle du robot."""
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Position actuelle du robot : {self.robot_position}")

        # Enregistrer la pose actuelle dans la trajectoire réelle parcourue
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.robot_position[0]
        pose.pose.position.y = self.robot_position[1]
        pose.pose.orientation.w = 1.0
        self.actual_trajectory.append(pose)
        self.publish_actual_path()
        rospy.loginfo(f"État du client d'action après modification de la : {self.move_base_client.get_state()}")
        rospy.loginfo(f"Fin d'une update de la position : {self.robot_position}")

    def update_goal_position(self, msg):
        """Mise à jour de la position de l'objectif et génération de la trajectoire sinusoïdale."""
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Nouvel objectif défini : {self.goal_position}")

            # Annulez tout objectif en cours
        current_state = self.move_base_client.get_state()
        if current_state in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            rospy.logwarn("Annulation de l'objectif en cours avant de définir un nouvel objectif.")
            self.move_base_client.cancel_goal()
            rospy.sleep(0.5)  # Pause pour permettre au client de se stabiliser

        # Réinitialiser l'index du point courant
        self.current_target_index = 0
        rospy.sleep(1)  # Attendre que move_base se stabilise
        
        if self.robot_position:
            # Générer et publier la trajectoire sinusoïdale
            self.publish_curved_path()
            # Commencer à suivre la trajectoire
            self.send_next_goal()
            rospy.loginfo(f"First point of the trajectory are follow")

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
        num_points = 5
        x_start, y_start = self.robot_position
        x_end, y_end = self.goal_position

        for i in range(1, num_points + 1):
            t = i / num_points
            x = (1 - t) * x_start + t * x_end
            y = (1 - t) * y_start + t * y_end + 1.5 * math.sin(2 * math.pi * t)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.planned_trajectory.append(pose)

        if len(self.planned_trajectory) < 2:
            rospy.logerr("La trajectoire générée est trop courte. Vérifiez les paramètres.")
            return

        # Publie la trajectoire sinusoïdale
        self.path_pub.publish(path_msg)
        rospy.loginfo("Trajectoire sinusoïdale publiée sur /planned_path")

        # Publie également sur le topic du planificateur global
        # self.global_plan_pub.publish(path_msg)
        rospy.loginfo("Trajectoire sinusoïdale publiée sur le topic du planificateur global /move_base/TrajectoryPlannerROS/global_plan")

    def send_next_goal(self):
        """Envoie le prochain point de la sinusoïde comme objectif."""
        rospy.loginfo(f"Go to the point {self.current_target_index} of the trajectory")

        if self.current_target_index < len(self.planned_trajectory):

            target_pose = self.planned_trajectory[self.current_target_index]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = target_pose.pose

            # # Vérifier et annuler l'objectif précédent si nécessaire
            # current_state = self.move_base_client.get_state()
            # rospy.loginfo(f"État du client d'action avant envoi : {current_state}")
            # if current_state in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            #     rospy.logwarn("Annulation de l'objectif précédent...")
            #     self.move_base_client.cancel_goal()
            #     self.move_base_client.wait_for_result(timeout=rospy.Duration(5.0))  # Attendez que l'annulation se termine
            #     rospy.sleep(0.2)  # Pause pour stabiliser l'état
            # elif current_state not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.RECALLED]:
            #     rospy.logwarn("Attente que l'état actuel soit stable avant d'envoyer un nouvel objectif.")
            #     self.move_base_client.wait_for_result(timeout=rospy.Duration(2.0))
            # elif current_state in [actionlib.GoalStatus.SUCCEEDED]:
            #      rospy.logwarn("OKKK")
            #      self.move_base_client.cancel_goal()
            #      self.move_base_client.wait_for_result(timeout=rospy.Duration(5.0)) 
                         
            # rospy.loginfo(f"Envoi du point {self.current_target_index} comme objectif.")
            # # self.move_base_client.send_goal(goal, done_cb=self.goal_reached_callback)
            # self.move_base_client.send_goal(goal, done_cb=self.goal_reached_callback)

            # Annulez l'objectif actuel pour éviter tout conflit
            rospy.loginfo("Annulation de l'objectif actuel (s'il existe).")
            self.move_base_client.cancel_goal()
            self.move_base_client.wait_for_result(timeout=rospy.Duration(2.0))

            # Réinitialisez le client d'action
            rospy.loginfo("Réinitialisation du client d'action.")
            self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.move_base_client.wait_for_server()

            # Envoyez le nouvel objectif
            rospy.loginfo(f"Envoi du point {self.current_target_index} comme nouvel objectif.")
            self.move_base_client.send_goal(goal, done_cb=self.goal_reached_callback)

            rospy.sleep(0.1)  # Pause courte pour s'assurer que l'objectif est actif
            new_state = self.move_base_client.get_state()
            rospy.loginfo(f"État après envoi : {new_state}")

            # Ajouter des logs pour vérifier l'état du client d'action
            rospy.sleep(0.1)  # Pause pour laisser le temps à l'action de démarrer

        else:
            rospy.loginfo("Tous les points de la trajectoire ont été atteints.")

    def goal_reached_callback(self, status, result):
        """Callback pour gérer l'atteinte de chaque objectif."""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Point {self.current_target_index} Reach.")

        #     # Publier le point atteint sur le topic "/reached_points"
            reached_point = self.planned_trajectory[self.current_target_index]
            self.reached_points.append(reached_point)

            self.reached_points_pub.publish(reached_point)

        #     self.current_target_index += 1
            
        #      # Vérifiez si la trajectoire est terminée
        #     if self.current_target_index < len(self.planned_trajectory):
        #         rospy.sleep(0.5)  # Add a short delay to ensure state transition
        #         rospy.loginfo(f"Passage au point {self.current_target_index}")
        #         self.send_next_goal()
        #     else:
        #         rospy.loginfo("Tous les points de la trajectoire ont été atteints.")
        # elif status in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED]:
        #     rospy.logwarn("Objectif annulé, tentative d'envoi du suivant.")
        #     self.current_target_index += 1
        #     rospy.sleep(0.2)
        #     self.send_next_goal()
        # else:
        #     rospy.logwarn("Objectif non atteint, réessai.")
        #     rospy.sleep(0.2)
        #     self.send_next_goal()

            # Réinitialisation pour éviter tout conflit
            rospy.loginfo("Réinitialisation après succès.")
            self.move_base_client.cancel_goal()
            rospy.sleep(0.5)  # Délai pour stabiliser l'état

            # Passer au prochain point
            self.current_target_index += 1

            if self.current_target_index < len(self.planned_trajectory):
                rospy.sleep(0.5)  # Délai pour la transition
                rospy.loginfo(f"Passage au point suivant : {self.current_target_index}")
                self.send_next_goal()
            else:
                rospy.loginfo("Tous les points de la trajectoire ont été atteints.")
        elif status in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED]:
            rospy.logwarn("Objectif annulé. Tentative d'envoi du prochain objectif.")
            self.send_next_goal()
        else:
            rospy.logwarn("État inattendu. Réessayer l'objectif actuel.")
            self.send_next_goal()


    def publish_actual_path(self):
        """Publie le chemin réel parcouru pour visualisation dans Rviz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.actual_trajectory
        self.actual_path_pub.publish(path_msg)
        rospy.loginfo("Chemin réel publié sur /actual_path")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_finding_algorithm")
    rospy.sleep(1)  # Pause pour s'assurer de la connexion
    rospy.loginfo("Démarrage de PathFindingAlgorithm.")
    pfa = PathFindingAlgorithm()
    pfa.run()