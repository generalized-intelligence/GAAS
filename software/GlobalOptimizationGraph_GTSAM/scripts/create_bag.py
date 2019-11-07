import rosbag
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

import time


bag = rosbag.Bag('test.bag', 'w')

gps_starting_time = None
gps_available_duration = 10 # out of cycle_duration
cycle_duration = 10 # assume a cycle to be 10 secs

gps_sub_topic = 'mavros/global_position/raw/fix'
slam_sub_topic = '/SLAM/pose_for_obs_avoid'

gps_out_topic = 'mavros/global_position/raw/fix'
slam_out_topic = '/SLAM/pose_for_obs_avoid'

def gps_callback(msg):
    global bag, gps_starting_time, gps_available_duration, cycle_duration, gps_out_topic

    if gps_starting_time is None:
        gps_starting_time = time.time()

    time_elapsed = time.time() - gps_starting_time
    if (time_elapsed <= gps_available_duration):
        print("Writting GPS info to bag!")
        bag.write(gps_out_topic, msg)
    elif (cycle_duration >= time_elapsed > gps_available_duration):
        print("GPS not available!")
        return
    else:
        gps_starting_time = time.time()


def slam_callback(msg):
    global bag

    # assume slam is always available
    bag.write(slam_out_topic, msg)



if __name__ == '__main__':

    rospy.init_node("gi_create_bag")

    rospy.Subscriber(gps_sub_topic, NavSatFix, gps_callback)
    rospy.Subscriber(slam_sub_topic, PoseStamped, slam_callback)

    while (not rospy.is_shutdown()):
        rospy.spin()

    bag.close()


