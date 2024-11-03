#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_custom_path():
    rospy.init_node('custom_path_publisher')
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)

    path = Path()
    path.header.frame_id = "odom"  # Le frame dans lequel les poses sont exprimées
    path.header.stamp = rospy.Time.now()

    # Générer des poses avec une courbe sinusoïdale
    for i in range(50):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = i * 0.1  # x augmente linéairement
        pose.pose.position.y = math.sin(i * 0.2)  # y suit une courbe sinusoïdale
        pose.pose.position.z = 0  # z est à 0 en 2D
        pose.pose.orientation.w = 1.0  # Orientation fixe pour simplifier
        path.poses.append(pose)

    # Publie le chemin calculé par ton algorithme
    rate = rospy.Rate(1)  # Publie à 1 Hz
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_custom_path()
    except rospy.ROSInterruptException:
        pass
