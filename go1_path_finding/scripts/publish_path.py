from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path(path):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    
    for position in path:
        pose = PoseStamped()
        pose.pose.position.x = position[0]  # x
        pose.pose.position.y = position[1]  # y
        pose.pose.orientation.w = 1.0       # Orientation neutre
        path_msg.poses.append(pose)
    
    path_pub.publish(path_msg)