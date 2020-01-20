import rospy
import sys 
sys.path.append('/home/robin/catkin_ws/devel/lib/python2.7/dist-packages')
from darknet_ros_msgs.msg import BoundingBoxes
from commander import Commander
import time
Kp=0.01
x_center=752/2
y_center=480/2
con=Commander()
def darknet_callback(data):
    if(data.bounding_boxes[0].xmax):
        x_error=y_center-(data.bounding_boxes[0].ymax+data.bounding_boxes[0].ymin)/2
        y_error=x_center-(data.bounding_boxes[0].xmax+data.bounding_boxes[0].xmin)/2
        con.move(Kp*x_error,Kp*y_error,z=0)
#rospy.init_node('yolo_tracking')
rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback)
 # spin() simply keeps python from exiting until this node is stopped
rate = rospy.Rate(60) #the rate will take care of updating the publisher at a specified speed - 50hz in this case
while(True):
    rate.sleep()
