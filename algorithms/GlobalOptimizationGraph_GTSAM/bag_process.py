import rospy
import rosbag
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, Image
import sys

topic_name_list = ['/gi/simulation/left/image_raw', '/gi/simulation/right/image_raw','/mavros/global_position/raw/fix','/mavros/imu/data']

if __name__ == '__main__':
    bag_in = rosbag.Bag(sys.argv[1])
    bag_out = rosbag.Bag('output.bag', 'w')
    init_ever = False
    for topic, msg, t in bag_in.read_messages():#topics=topic_name_list):
        if not init_ever:
            init_ever = t
        if(topic!='/mavros/global_position/raw/fix'):
            bag_out.write(topic,msg,t=t)
        else:
            if t> init_ever + rospy.Duration(75) and t<init_ever+rospy.Duration(120):
                pass
            else:
                bag_out.write(topic,msg,t=t)
    bag_in.close()
    bag_out.close()

