from mavros_msgs.msg import Trajectory
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading

if __name__ == '__main__':
    rospy.init_node("test")

    local_target_pub = rospy.Publisher('/mavros/trajectory/generated', Trajectory, queue_size=1)

    t = Trajectory()
    t.header.stamp = rospy.Time.now()
    t.type = 0

    t.point_1.position.x = 2
    t.point_1.position.y = 0
    t.point_1.position.z = 2
    t.point_1.velocity.x = 0
    t.point_1.velocity.y = 2
    t.point_1.velocity.z = 0

    t.point_2.position.x = 2
    t.point_2.position.y = 2
    t.point_2.position.z = 2
    t.point_2.velocity.x = -2
    t.point_2.velocity.y = 0
    t.point_2.velocity.z = 0

    t.point_3.position.x = 0
    t.point_3.position.y = 2
    t.point_3.position.z = 2
    t.point_3.velocity.x = 0
    t.point_3.velocity.y = -2
    t.point_3.velocity.z = 0

    t.point_4.position.x = 0
    t.point_4.position.y = 0
    t.point_4.position.z = 2
    t.point_4.velocity.x = 0
    t.point_4.velocity.y = 0
    t.point_4.velocity.z = 0

    t.point_valid = [True, True, True, True, False]
    t.time_horizon = [2, 2, 2, 2, 0]

    local_target_pub.publish(t)

    rospy.spin()




