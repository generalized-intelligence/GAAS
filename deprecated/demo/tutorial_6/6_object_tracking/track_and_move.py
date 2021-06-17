#encoding=utf-8
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import message_filters

import sys 
sys.path.append('/home/gi/Tracking/KCF/build/devel/lib/python2.7/dist-packages')
from ros_kcf.srv import InitRect
from std_msgs.msg import Int32MultiArray
from commander import Commander
import time
import config


#Keep the object in the center.
class SimpleTrackAndMove:
    def __init__(self, resolution,  K, left_topic='/gi/simulation/left/image_raw', right_topic='/gi/simulation/right/image_raw', 
                        object_position_topic='/track_rect_pub' ,move_threshold=0.3, altitude=3000, stereo=None, baseline=None, ):

        self.fx = float(K[0][0])
        self.fy = float(K[1][1])

        # self.idx = 0
        self.con = Commander()
        time.sleep(0.1)

        self.left_sub = message_filters.Subscriber(left_topic, Image)
        self.right_sub = message_filters.Subscriber(right_topic, Image)
        self.object_position_sub = message_filters.Subscriber(object_position_topic, Int32MultiArray)
        ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub, self.object_position_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.update_new_position)

        self.prev_position = None
        self.cur_position = None
        self.width = resolution[0]
        self.hight = resolution[1]

        self.camera_center = self.get_center((0,0, self.width, self.hight))
        self.move_threshold = move_threshold
        self.altitude = altitude

        self.stereo = stereo
        if stereo is not None:
            assert baseline is not None
        self.baseline = baseline

        self.bridge = CvBridge()

        rospy.spin()


    def check_border(self, box):
        x1 = max(box[0],0) if box[0] <= self.width else self.width
        y1 = max(box[1],0) if box[1] <= self.hight else self.hight
        x2 = max(box[2],0) if box[2] <= self.width else self.width
        y2 = max(box[3],0) if box[3] <= self.hight else self.hight
        return (x1, y1, x2, y2)

    def get_center(self, box):
        x1 = box[0]
        y1 = box[1]
        x2 = box[2]
        y2 = box[3]
        return (abs(x2+x1)/2., abs(y2+y1)/2.)
    
    def update_new_position(self, left, right, track_result):
        left_img = self.bridge.imgmsg_to_cv2(left, "mono8")
        right_img = self.bridge.imgmsg_to_cv2(right, "mono8")

        box = track_result.data
        self.cur_position = self.check_border(box)
        # if self.idx%5 == 0:
        #     left_img_rgb = self.bridge.imgmsg_to_cv2(left, "bgr8")
        #     left_img_rgb = cv2.rectangle(left_img_rgb, (self.cur_position[0],self.cur_position[1]), (self.cur_position[2],self.cur_position[3]), (255,0,0))
        #     cv2.imwrite('/home/gi/Documents/img/{}.png'.format(str(self.idx)), left_img_rgb)
        # self.idx+=1

        self.update_altitude(left_img, left_img, track_result)

        new_move = self.get_next_move()
        if new_move is not None:
            print('Move:',new_move)
            self.con.move(new_move[0],new_move[1],new_move[2])
            time.sleep(0.1)

    def update_altitude(self, left, right, track_result):
        if self.stereo is not None:
            disparity = stereo.compute(imgL,imgR)
            disparity = cv2.convertScaleAbs(disparity, disparity, alpha = 1./16)
            mean_disparity = np.mean(disparity[track_result[1]:track_result[3], track_result[0]:track_result[2]])
            mean_depth = self.baseline* self.fx / mean_disparity 
            self.altitude = mean_depth
        else:
            pass
    
    def get_point_dist(self, p1, p2):
        return np.sqrt(np.sum(np.square(np.asarray(p1)-np.asarray(p2))))
    
    def get_dist(self, a, b):
        return np.sqrt(np.sum(np.square(a)+np.square(b)))
    
    def get_next_move(self):
        print('Current position box: ',self.cur_position)
        cur_center = self.get_center(self.cur_position)

        #|----^ X---|
        #|----|-----|
        #|Y---|-----|
        #<————+-----|
        #|----------|
        #|----------|
        y = -(cur_center[0]- self.camera_center[0])*self.altitude/self.fx
        x = -(cur_center[1] - self.camera_center[1])*self.altitude/self.fy

        if self.get_dist(x,y)>self.move_threshold:
            return (x/1000.,y/1000.,0)
        else:
            return None



if __name__ == '__main__':
    K = config.K 
    resolution = config.resolution
    left_topic = config.left_topic
    right_topic = config.right_topic
    object_position_topic = config.object_position_topic

    SimpleTrackAndMove(resolution, K, left_topic=left_topic, right_topic=right_topic, object_position_topic=object_position_topic)