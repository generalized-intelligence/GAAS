#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

# AirSim Python API

import airsim

import rospy

# ROS Image message
from sensor_msgs.msg import Image

def airpub():
    pub_left = rospy.Publisher("/gi/simulation/left/image_raw", Image, queue_size=1)
    pub_right=rospy.Publisher("/gi/simulation/right/image_raw", Image, queue_size=1)
    rospy.init_node('airsim_info', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
        # get camera images from the car
        Imu = client.getImuData()
        responses = client.simGetImages([
            airsim.ImageRequest("front-left", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("front-right", airsim.ImageType.Scene, False, False)
        ])
        img_dic={}
        for idx, response in enumerate(responses):
            img_rgb_string = response.image_data_uint8
            img_dic[idx]=img_rgb_string
        
        # Populate image message
        msg_left=Image()
        msg_left.header.stamp = rospy.Time.now()
        msg_left.header.frame_id = "frameId"
        msg_left.encoding = "rgb8"
        msg_left.height = 480  # resolution should match values in settings.json
        msg_left.width = 752
        msg_left.data = img_dic[0]
        msg_left.is_bigendian = 0
        msg_left.step = msg_left.width * 3
        msg_right = Image()
        msg_right.header.stamp = rospy.Time.now()
        msg_right.header.frame_id = "frameId"
        msg_right.encoding = "rgb8"
        msg_right.height = 480  # resolution should match values in settings.json
        msg_right.width = 752
        msg_right.data = img_dic[1]
        msg_right.is_bigendian = 0
        msg_right.step = msg_left.width * 3
        # log time and size of published image
        rospy.loginfo(len(response.image_data_uint8))
        # publish image message
        pub_left.publish(msg_left)
        pub_right.publish(msg_right)
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        print('wrong')
        pass
