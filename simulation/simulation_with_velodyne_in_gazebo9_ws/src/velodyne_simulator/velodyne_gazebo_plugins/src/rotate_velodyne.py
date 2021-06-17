#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np

def main(mean, stddev):
    rate = rospy.Rate(10)
    pub = rospy.Publisher("velodyne_controller/command", Float64, queue_size=1)
    while not rospy.is_shutdown():
        newMsg = Float64()
        newMsg.data = np.random.normal(mean, stddev, 1)[0]
        pub.publish(newMsg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("rotate_velodyne")
    mean = rospy.get_param("~rotate_velodyne/mean")
    stddev = rospy.get_param("~rotate_velodyne/stddev")
    print("Got parameter [rotate_velodyne/mean]: {}".format(mean))
    print("Got parameter [rotate_velodyne/stddev]: {}".format(stddev))
    try:
        main(mean, stddev)
    except rospy.ROSInterruptException:
        pass