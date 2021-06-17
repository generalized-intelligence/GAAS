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
        rospy.init_node("uav1_commander_node")
        rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('/uav1/gi/set_pose/position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('/uav1/gi/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('/uav1/gi/set_activity/type', String, queue_size=10)

        self.local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.local_pose_callback)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))


    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    
    # land at current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))


    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))


    def set_pose(self, x=0, y=0, z=2, BODY_FLU = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'base_link'

        else:
            pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose


if __name__ == "__main__":
    
    con = Commander()
    time.sleep(2)
    i=1
    while True:
        con.move(50*math.cos(2*math.pi*i/100),50*math.sin(2*math.pi*i/100), con.local_pose.pose.position.z ,BODY_OFFSET_ENU=False)
        if abs(50*math.cos(2*math.pi*i/100)-con.local_pose.pose.position.x)<2 and abs(50*math.sin(2*math.pi*i/100)-con.local_pose.pose.position.y)<2:
            i=i+1
        time.sleep(0.02)