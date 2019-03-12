#encoding=utf-8


import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
publisher = None
index = 0
def callback_func(msg_in):
    global index
    new_marker = Marker()
    new_marker.header.frame_id="map"
    new_marker.id = index
    new_marker.type = 0
    index+=1
    new_marker.pose.position.x = msg_in.pose.position.x
    new_marker.pose.position.y = msg_in.pose.position.y
    new_marker.pose.position.z = msg_in.pose.position.z
    new_marker.color.r = 1
    new_marker.color.a = 1
    new_marker.scale.x = 0.1
    new_marker.scale.y = 0.1
    new_marker.scale.z = 0.1
    publisher.publish(new_marker)

if __name__ == '__main__':
    global publisher
    rospy.init_node("pose_visualize_tx2")
    publisher = rospy.Publisher("/tx2_visualized_marker",Marker)
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback_func)
    rospy.spin()
