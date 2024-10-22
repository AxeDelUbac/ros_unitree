#!/usr/bin/env python3

import rospy
#from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np  # Assurez-vous d'importer numpy si vous l'utilisez

def scan_callback(scan: LaserScan):

    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment

    #rospy.loginfo("seq: %d, stamp: %s, frame_id: %s", scan.header.seq, scan.header.stamp,scan.header.frame_id)
    #rospy.loginfo("angle_min: %f, angle_max: %f", angle_min, angle_max)
    #rospy.loginfo("angle_increment: %f, time_increment: %f, scan_time: %f", scan.angle_increment, scan.time_increment, scan.scan_time)

    ranges = scan.ranges  # Liste des distances (en mètres)
    num_measurements = len(ranges)
    rospy.loginfo("Number of measurements: %d", num_measurements)
    rospy.loginfo("First measurement: %f", ranges[0])
    
    ranges_array = np.array(ranges)
    min_distance = np.min(ranges_array)
    min_index = np.argmin(ranges_array)
    angle_at_min = angle_min + min_index * angle_increment

    rospy.loginfo("Minimum distance: %f meter, at angle: %f radians", min_distance, angle_at_min)

    intensities = scan.intensities  # Liste des intensités

    if len(intensities) > 0:
        rospy.loginfo("First intensity: %f", intensities[0])
    else:
        rospy.loginfo("Intensities not provided by this LIDAR.")





if __name__ == '__main__':
    rospy.init_node("go1_astar_node")

    rospy.loginfo("Node has been started")

    sub = rospy.Subscriber("/go1_gazebo/scan", LaserScan, callback=scan_callback)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)
    
    rospy.spin()

    pub= rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    rate = rospy.Rate()

    while not rospy.is_shutdown():
        
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)

        rate.sleep()