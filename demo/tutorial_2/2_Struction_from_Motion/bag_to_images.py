import os
import argparse
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag


# extract images froma  rosbag
# input params: --bag bag_name
# 		--output_path place_where_you_want_to_save_images
#		--image_topic image_topic_that_you_want_to_sub_and_save
def bag2image():
    parser = argparse.ArgumentParser(description="rosbag to images")
    parser.add_argument("--bag")
    parser.add_argument("--output_path")
    parser.add_argument("--image_topic")
    args = parser.parse_args()
    
    print "bag file: ", args.bag
    print "output image path: ", args.output_path
    print "output image path: ", args.image_topic

    bag = rosbag.Bag(args.bag, "r")
    bridge = CvBridge()
    count = 0

    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(os.path.join(args.output_path, "image_%i.png" % count), cv_img)
        print "procesing image index: ", count
        count += 1

    bag.close()
 

if __name__ == '__main__':
    bag2image()
