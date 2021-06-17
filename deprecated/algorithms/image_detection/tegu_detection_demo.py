
import sys
#sys.path.append('..')
from Tegu.Network.YOLOv3.API import YOLOv3_Dataloader, YOLOv3_Model
import os
import cv2
import numpy as np
from Util.mylogger import MyLogger

import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



pModel = None
pPublisher = None
bridge = CvBridge()


def get_range_from_bbox(imgl,imgr,img_l_bbox):
    #step1.generate grid and sample gftt.
    #step2.calc disparity via OptFlow.
    #step3.estimate depth.
    pass


def image_callback(img_msg):
    if pModel is None:
        print("Model load failed.")
    img = bridge.imgmsg2cv2(img_msg)
    result = pModel.predict(img)
    output_msg = str(result)
    if pPublisher is None:
        print("Publish failed.")
    pPublisher.publish(output_msg)


def load_model(model_path):
    pModel = YOLOv3_Model.load(model_path) # need a api.


def main():
    print("Usage: demo.py [model path] [image topic] [output topic]")
    model_path = sys.argv[1]
    image_topic_name = sys.argv[2]
    output_topic_name = sys.argv[3]
    if(len(sys.argv)<4):
        print("arg error.")
        return
    pModel = load_model(model_path)
    rospy.init("GAAS_ImageDetection_TEGU_demo"+model_path)  # which supports multi process.
    pPublisher = rospy.Publisher(output_topic_name)
    rospy.Subscribe(image_topic_name,10,callback)
    rospy.spin()

if __name__ == '__main__':
    main()
