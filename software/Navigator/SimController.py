#encoding=utf-8

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Controller:
    def __init__(self):
        rate = rospy.Rate(20)
        self.local_setposition_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        self.local_setorientation_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)


    def set_pose(self, x=0, y=0, z=2, abs_mode = True):

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        print("(x,y,z):", (x, y, z))

        if not abs_mode:
            pose.header.frame_id = "base_link"  # set to relative mode.

        return pose

    def _takeoff_ned_xyz_relative(self, x, y, z):
        self.local_setposition_pub.publish(self.set_pose(x, y, z, abs_mode=False))

    def _takeoff_ned_xyz_abs(self, x, y, z):
        self.local_setposition_pub.publish(self.set_pose(x, y, z))

    def _turn_relative_to_takeoff_abs(self, degree):
        self.local_setorientation_pub.publish(degree)

    def mav_move(self, x, y, z, yaw=None, abs_mode=True):

        # x, y, z = self.ned2enu(x, y, z)

        if abs_mode:
            self._takeoff_ned_xyz_abs(x, y, z)
        else:
            self._takeoff_ned_xyz_relative(x, y, z)
        if yaw:
            self._turn_relative_to_takeoff_abs(yaw)


    # given a point in NED, return corresponding coord in ENU
    def ned2enu(self, x, y, z):

        ENU_x = -y
        ENU_y = x
        ENU_z = z

        return ENU_x, ENU_y, ENU_z


if __name__ =='__main__':

    rospy.init_node("sim_controller_node")
    con = Controller()

    time.sleep(5)
    con.mav_move(0, 0, 30, abs_mode=True)

    time.sleep(20)
    con.mav_move(20, 0, 30, abs_mode=True)

    time.sleep(20)
    con.mav_move(40, 0, 30, abs_mode=True)

    time.sleep(20)
    con.mav_move(60, 0, 30, abs_mode=True)

    time.sleep(20)
    #con._turn_relative_to_takeoff_abs(90)
    #time.sleep(20)
    con.mav_move(60, -20, 30, abs_mode=True)

    time.sleep(20)
    #con._turn_relative_to_takeoff_abs(-90)
    time.sleep(20)
    con.mav_move(60, -40, 30, abs_mode=True)

    time.sleep(20)
    con.mav_move(60, -80, 30, abs_mode=True)
    
    time.sleep(20)
    con.mav_move(60, -80, 10, abs_mode=True)

    time.sleep(6)
    con.mav_move(60, -80, 2, abs_mode=True)

    con._turn_relative_to_takeoff_abs(0)



