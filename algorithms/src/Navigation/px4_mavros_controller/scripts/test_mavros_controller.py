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
        self.position_target_pub = rospy.Publisher('/gaas/navigation/target_position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('/gaas/navigation/target_enu_yaw', Float32, queue_size=10)
        #self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10)


    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))


    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    
    # land at current position
    #def land(self):
    #    self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    #def hover(self):
    #    self.custom_activity_pub.publish(String("HOVER"))


    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))


    def set_pose(self, x=0, y=0, z=2, BODY_FLU = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'lidar'

        else:
            pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose


def start_mavros_controller():
    mavros_controller = px4_mavros_run.Px4Controller()
    mavros_controller.start()
 
    pass

if __name__ == "__main__":
    #step<1> test relative body FLU.
    time.sleep(1)
    con = Commander()
    print ("Commander initiated!")
    time.sleep(1)
    print("Take off to height 4m!")

    con.move(0,0,4)
    time.sleep(4)
    con.turn(0)
    time.sleep(5)
    print("Move 4,0,0!")
    con.move(4, 0, 0)
    time.sleep(10)

    print("Move 0,4,0!")
    con.move(0, 4, 0)
    time.sleep(10)


    print("turn 90deg!")
    con.turn(90)
    time.sleep(5)
    con.turn(-90)
    print("turn -90deg!")
    time.sleep(5)
    #print("Land!")
    #con.land()
    #time.sleep(5)
    print("finished.")
    con.turn(0)
    time.sleep(4)
    con.return_home(0.1)
    time.sleep(10)

    #step<2> test map NWU.

    con.move(4,0,0,False)
    time.sleep(5)
    con.move(0,4,0,False)
    time.sleep(5)
    con.return_home(0.1)
    time.sleep(5)

    exit(0)


