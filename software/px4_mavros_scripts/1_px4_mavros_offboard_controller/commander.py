import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander:
    def __init__(self):
        rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10)


    def move(self, x, y, z):
        self.position_target_pub.publish(self.set_pose(x, y, z))

    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    
    # land in current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))


    def set_pose(self, x=0, y=0, z=2, BODY_OFF_SET_NED = True):
        pose = PoseStamped()
        
        if BODY_OFF_SET_NED:
            pose.header.frame_id = 'frame.body'
        else:
            pose.header.frame_id = 'frame.local_ned'        

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

   
con = Commander()
con.move(1,0,0)
time.sleep(2)
con.turn(90)
time.sleep(2)
con.land()


