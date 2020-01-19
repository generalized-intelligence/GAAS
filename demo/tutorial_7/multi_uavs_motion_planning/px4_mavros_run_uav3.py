import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading


class Px4Controller:

    def __init__(self):

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 10
        self.local_enu_position = None

        self.cur_target_pose = PoseStamped()
        self.cur_target_twist=TwistStamped()
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.flag = 0
        self.state = None

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/uav3/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/uav3/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/uav3/mavros/imu/data", Imu, self.imu_callback)

        self.set_target_position_sub = rospy.Subscriber("/uav3/gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_velocity_sub = rospy.Subscriber("/uav3/gi/set_pose/velocity", Vector3Stamped, self.set_target_velocity_callback)
        self.set_target_yaw_sub = rospy.Subscriber("/uav3/gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("/uav3/gi/set_activity/type", String, self.custom_activity_callback)


        '''
        ros publishers
        '''
        self.pose_target_pub = rospy.Publisher('/uav3/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.twist_target_pub = rospy.Publisher('/uav3/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/uav3/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/uav3/mavros/set_mode', SetMode)


        print("Px4 Controller Initialized!")


    def start(self):
        rospy.init_node("uav3_offboard_node")

        self.cur_target_pose = self.construct_pose_target(x=self.local_pose.pose.position.x, y=self.local_pose.pose.position.y, z=self.takeoff_height)

        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(20):
            self.pose_target_pub.publish(self.cur_target_pose)
            self.twist_target_pub.publish(self.cur_target_twist)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)


        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        '''
        main ROS thread
        '''
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            if(self.flag==0):
                self.pose_target_pub.publish(self.cur_target_pose)
            else:
                self.twist_target_pub.publish(self.cur_target_twist)
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if(self.disarm()):

                    self.state = "DISARMED"


            time.sleep(0.1)


    def construct_pose_target(self, x=0, y=0, z=0):
        target_raw_pose = PoseStamped()
        target_raw_pose.header.stamp = rospy.Time.now()
        target_raw_pose.pose.position.y = x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''
    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False


    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg



    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)

        self.received_imu = True


    def gps_callback(self, msg):
        self.gps = msg


    def body2enu(self, body_target_x, body_target_y, body_target_z):

        ENU_x = body_target_y
        ENU_y = - body_target_x
        ENU_z = body_target_z

        return ENU_x, ENU_y, ENU_z

    def body2enu_velocity(self, body_target_vx, body_target_vy, body_target_vz):

        ENU_vx = body_target_vy
        ENU_vy = - body_target_vx
        ENU_vz = body_target_vz

        return ENU_vx, ENU_vy, ENU_vz

    def BodyOffsetENU2FLU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    def BodyOffsetENU2FLU_Velocity(self, msg):
        FLU_vx = msg.vector.x * math.cos(self.current_heading) - msg.vector.y * math.sin(self.current_heading)
        FLU_vy = msg.vector.x * math.sin(self.current_heading) + msg.vector.y * math.cos(self.current_heading)
        FLU_vz = msg.vector.z

        return FLU_vx, FLU_vy, FLU_vz

    def set_target_position_callback(self, msg):
        self.flag = 0
        #print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_OFFSET_ENU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            #print("body FLU frame")

            FLU_x, FLU_y, FLU_z = self.BodyOffsetENU2FLU(msg)

            body_x = FLU_x + self.local_pose.pose.position.x
            body_y = FLU_y + self.local_pose.pose.position.y
            body_z = FLU_z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_pose_target(x=body_x,
                                                         y=body_y,
                                                         z=body_z,
                                                         )


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            #print("local ENU frame")

            ENU_x, ENU_y, ENU_z = self.body2enu(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

            self.cur_target_pose = self.construct_pose_target(x=ENU_x,y=ENU_y,z=ENU_z)


    def set_target_velocity_callback(self, msg):
        self.flag = 1
        #print("Received New Velocity Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_OFFSET_ENU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            #print("body FLU frame")

            FLU_vx, FLU_vy, FLU_vz = self.BodyOffsetENU2FLU_Velocity(msg)
            self.cur_target_twist.twist.linear.x=FLU_vx
            self.cur_target_twist.twist.linear.y=FLU_vy
            self.cur_target_twist.twist.linear.z=FLU_vz
            #print(self.cur_target_pose)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            #print("local ENU frame")

            ENU_vx, ENU_vy, ENU_vz = self.body2enu_velocity(msg.x, msg.y, msg.z)

            #self.cur_target_pose = self.construct_twist_target(vx=ENU_vx,vy=ENU_vy,vz=ENU_vz)
    
    '''
    Receive A Custom Activity
    '''

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_pose_target(x=self.local_pose.pose.position.x,
                                                         y=self.local_pose.pose.position.y,
                                                         z=0.1,
                                                         )

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_pose_target(x=self.local_pose.pose.position.x,
                                                     y=self.local_pose.pose.position.y,
                                                     z=self.local_pose.pose.position.z,
                                                     )

    '''
    return yaw from current IMU
    '''
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def hover(self):

        self.cur_target_pose = self.construct_pose_target(x=self.local_pose.pose.position.x,
                                                     y=self.local_pose.pose.position.y,
                                                     z=self.local_pose.pose.position.z,
                                                     )

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':

    con = Px4Controller()
    con.start()
    time.sleep(2)
