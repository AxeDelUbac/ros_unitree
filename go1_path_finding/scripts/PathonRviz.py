#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path():
    rospy.init_node('path_publisher')
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)

    path = Path()
    path.header.frame_id = "map"
    
    # Création de plusieurs poses (points du chemin)
    for i in range(10):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = i  # Exemple de chemin en ligne droite
        pose.pose.position.y = i
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    # Publie le chemin périodiquement
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
