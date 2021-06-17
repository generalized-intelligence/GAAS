#encoding=utf-8

'''
        project overview:

	Subscribe:
		1.slam pose(global/local pose) *
		2.octomap_server/global map
		3.local pointcloud/local octomap
		4.target input(semantic target/visual pose target/gps target)
	Publish:
		1.Mavros(amo) Command
		2.Navigator status

	Algorithms:
		1.D*
		2.state transfer
		3.position->position PID controller
		4.global/semantic/visual target to local pose
'''

import threading
import time
from path_optimization.path_pruning import PathPruning
# for ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, NavSatFix, PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker,MarkerArray
# for mavros
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget#, Command
from mavros_msgs.srv import CommandBool, SetMode


# for octomap
from octomap_msgs.msg import Octomap, OctomapWithPose, octomap_msgs


# other useful utilities
#from pyquaternion import Quaternion
import pyquaternion

import astar.astar
import astar.driver

import time
import math
from enum import Enum
import thread
#from queue import Queue

#from Pos2PosController import Pos2PosController as Controller  # TODO:re-implement this.
from SimController import Controller as Controller
import DiscreteGridUtils

import numpy as np

# define system status
class status(Enum):
    INITIALIZED = 1

    LOOKING_FOR_PATH = 2
    LOOKING_FOR_PATH_SUCCEED = 3
    LOOKING_FOR_PATH_FAILED = 4

    GOING_TO_TARGET = 5
    GOING_TO_VISION_TARGET = 6
	

def dist(pos1,pos2):
    if not pos1 or not pos2:
        return False, 0
    else:
        return True, reduce(lambda x,y:x+y,map(lambda i:(pos1[i]-pos2[i])**2,[0,1,2]))

class Navigator:
    
    def __init__(self,config_file_path = None):
        if config_file_path:
            pass


        rospy.init_node("gi_navigator_node")
        self.dg = DiscreteGridUtils.DiscreteGridUtils(grid_size=0.2)#0.2)
        self.rate = rospy.Rate(50)
        self.driver = astar.driver.Driver()
        self.controller = Controller()
        self.mavros_state = "OFFBOARD"
        self.set_status(status.INITIALIZED)

        self.cur_command_id = 0
        self.prev_command_id = 0
        self.cur_target_position=None

        self.task_id = -1
        self.obstacle_set_mutex = threading.Lock()  # mutex.acquire(timeout);mutex.release()
        self.nav_command_mutex = threading.Lock()  # for nav command in dstar and ros high level command.
        self.local_pose = None
        t1 = threading.Thread(target=self.ros_thread)
        t1.start()
        
        self.navigator_status_pub = rospy.Publisher('/gi/navigator_status', String, queue_size=10)
        self.path_plan_pub = rospy.Publisher('/gi/navi_path_plan',MarkerArray,queue_size=10)
        #t2 = thread.start_new_thread(self.Dstar_thread, ())

        #self.keep_navigating()

        self.path = []
        self.path_prune = PathPruning(obstacle_distance=8)
        time.sleep(2)


    '''
    Navigating thread    
    '''
    def keep_navigating(self):

        # for debug:
        #self.algo = astar.astar.A_star((1,0,0))

        while self.mavros_state == "OFFBOARD" and not(rospy.is_shutdown()):

            # print ('Inside outer loop!')
            #print ("Navigator state: ", self.STATUS.data, "Mavros state: ", self.mavros_state)
            relative_pos = (0, 0, 0)

            end_pos = self.get_latest_target()

            current_pos = self.get_current_pose() # TODO:fix this.
            if current_pos is None:
                #print ('current pose not valid!')
                continue

            while current_pos != end_pos and not self.navi_task_terminated() and not(rospy.is_shutdown()):  # Till task is finished:
                # print ('Inside inner loop!')
                current_pos = self.get_current_pose()
                self.algo = astar.astar.A_star(end_pos)
                print ('Move 1 step')

                obstacle_map = self.driver.get_obstacles_around()  # TODO:加入障碍记忆.
                print ('From ', current_pos)
                t1 = time.time()
                self.driver.algo = astar.astar.A_star(end_pos)
                self.path = self.algo.find_path(current_pos, self.driver.get_obstacles_around())
                t2 = time.time()
                print('A* time cost:', (t2 - t1))

                if not self.path:
                    #TODO set status
                    print ('No path found!')
                    self.do_hover()  # TODO
                    time.sleep(0.05)  # TODO
                else:
                    # Path found. keep state machine and do task step by step.

                    self.collear_check_path = self.path_prune.remove_collinear_points(self.path)
                    self.bresenham_check_path = self.path_prune.path_pruning_bresenham3d(self.collear_check_path, obstacle_map)

                    #publish raw path plan.
                    m_arr = MarkerArray()
                    marr_index = 0
                    for next_move in self.path:
                        point = self.dg.discrete_to_continuous_target((next_move[0],next_move[1],next_move[2]))
                        mk = Marker()
                        mk.header.frame_id="map"
                        mk.action=mk.ADD
                        mk.id=marr_index
                        marr_index+=1
                        mk.color.r = 1.0
                        mk.color.a = 1.0
                        mk.type=mk.CUBE
                        mk.scale.x = 0.3
                        mk.scale.y = 0.3
                        mk.scale.z = 0.3
                        mk.pose.position.x = point[0]
                        mk.pose.position.y = point[1]
                        mk.pose.position.z = point[2]
                        m_arr.markers.append(mk)
                    self.path_plan_pub.publish(m_arr)

                    for next_move in self.bresenham_check_path:
                        self.path_plan_pub.publish(m_arr)
                        if self.navi_task_terminated():
                            break

                        print ('current_pos:', current_pos)
                        next_pos = next_move
                        relative_pos = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1],
                                        next_pos[2] - current_pos[2])
                        print ('next_move : ', next_move)
                        print ("relative_move : ", relative_pos)
                        print ("next_pose: ", next_pos)
                        if not self.driver.algo.is_valid(next_pos, self.driver.get_obstacles_around()):
                            print ('Path not valid!')
                            break
                        self.current_pos = next_pos

                        #axis transform
                        relative_pos_new = (-relative_pos[0], -relative_pos[1], relative_pos[2])

                        #self.controller.mav_move(*relative_pos_new,abs_mode=False) # TODO:fix this.
                        print ('mav_move() input: relative pos=',next_pos)
                        self.controller.mav_move(*self.dg.discrete_to_continuous_target((next_pos[0],next_pos[1],next_pos[2])), abs_mode=True)  # TODO:fix this.

                        current_pos = self.get_current_pose()
                        time.sleep(2)
                        predict_move = (self.current_pos[0] + relative_pos[0], self.current_pos[1] + relative_pos[1],
                                        self.current_pos[2] + relative_pos[2])
                        print ("predict_move : ", predict_move)


                        if not self.algo.path_is_valid(self.bresenham_check_path, self.driver.get_obstacles_around()):
                            print ('Path conflict detected!')
                            break

            time.sleep(0.05) # wait for new nav task.
            '''
            if self.found_path:

                print("Found path!")

                target_position = self.cur_target_position

                result = False
                while result is False:
                    result = self.mav_move(target_position[0], target_position[1], target_position[2])


                print("Reached Position: ", target_position[0], target_position[1], target_position[2])
                print("Finished Current Path")

            time.sleep(0.2)
            '''


        print("Mavros not in OFFBOARD mode, Disconnected!")


    '''
    move quad in body frame
    '''

    def distance(self, p1, p2):
        x_distance = (p2[0] - p1[0])**2
        y_distance = (p2[1] - p1[1])**2
        z_distance = (p2[2] - p1[2])**2

        return np.sqrt(x_distance + y_distance + z_distance)


    def remove_collinear_points(self, original_path):

        new_path = []
        print ("original_path length: ", len(original_path))

        length = len(original_path) - 2
        new_path.append(original_path[0])
        # new_path.append(original_path[-1])

        print(original_path)

        for i in range(length):

            distance13 = self.distance(original_path[i+2], original_path[i])
            distance12 = self.distance(original_path[i+1], original_path[i])
            distance23 = self.distance(original_path[i+2], original_path[i+1])

            print("distance13 - distance12 - distance23 :", distance13 - distance12 - distance23 )
            if abs(distance13 - distance12 - distance23)  < 0.001:
                # print ("points collinear")
                continue

            else:
                print(original_path[i+1])
                print("not found collinear point")
                new_path.append(original_path[i+1])

        print("new path length: ", len(new_path))
        print(new_path)

        return new_path




    def terminate_navigating(self):
        #TODO
        pass

    def resume_navigating(self):
        #TODO
        pass

    def do_hover(self):
        #TODO
        pass


    def set_target_postion(self, target_position):
        self.found_path = True
        self.cur_target_position = self.dg.continuous_to_discrete(target_position)
        print("Current target position in grid: ", self.cur_target_position)
        #print("Set Current Position to: ", target_position[0], target_position[1], target_position[2])

    def get_latest_target(self):
        return self.cur_target_position

    def set_vision_target(self):
        self.set_status(status.GOING_TO_VISION_TARGET)
        self.set_target_position(xxxxx) #TODO
        pass

    def navi_task_terminated(self):
        if dist(self.local_pose,self.cur_target_position) <0.25:  #TODO: or stop flag is set.
            return True
        else:
            return False

    '''
    Dstar Thread
    
    def Dstar_thread(self):
        while not rospy.is_shutdown():
            while status!= xxx:# TODO
                next_move = xxx
                return next_move'''

    '''##For test:
        target = [0.5, 0.5, 0.5]
        self.set_target_postion(target)
        pass'''




    '''
    ROS thread
    responsible for subscribers and publishers
    '''
    def ros_thread(self):
        print('ros_thread spawn!!!!')
        self.octomap_msg = None

        # subscribers
        self.slam_sub = rospy.Subscriber("/gi/slam_output/pose", PoseStamped, self.slam_pose_callback)
        self.vision_target_sub = rospy.Subscriber("/gi/visual_target/pose", PoseStamped, self.vision_target_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera/left/point_cloud", PointCloud, self.point_cloud_callback)
        self.octomap_cells_vis = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_update_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)


        # publishers
        #self.mavros_control_pub = rospy.Publisher('mavros/Command', Command, queue_size=10)

        self.set_status(status.INITIALIZED)

        rospy.spin()


    '''
    ROS callbacks
    '''
    def slam_pose_callback(self, msg):
        self.slam_pose = msg

    def vision_target_callback(self, msg):
        self.vision_target = msg
        #print("Received New Vision Target!")

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        #print(msg.mode, type(msg.mode))
        self.navigator_status_pub.publish(self.STATUS)

    def point_cloud_callback(self, msg):
        self.current_point_cloud = msg

    def octomap_update_callback(self, msg):  # as pointcloud2.
        obs_set = set()
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #print " x : %f  y: %f  z: %f" % (p[0], p[1], p[2])
            point = self.dg.continuous_to_discrete((p[0],p[1],p[2]))
            #print ('point:',point)
            obs_set.add(point)
        acquired = self.obstacle_set_mutex.acquire(True)  # blocking.
        if acquired:
            #print('octomap updated!')
            self.driver.set_obstacle_set(obs_set)
            self.obstacle_set_mutex.release()
            return
        else:
            print ('Lock not acquired!')

    def local_pose_callback(self, msg):
        pose_ = msg.pose.position #TODO:do fusion with visual slam.
        self.local_pose = self.dg.continuous_to_discrete((pose_.x,pose_.y,pose_.z))
        #print ('local_pose set!!!')

    def get_local_pose(self): # in mavros axis.for command.
        #print ('self.local_pose',self.local_pose)
        return self.local_pose

    def get_current_pose(self): # current pose:slam pose(in world axis)
        return self.get_local_pose() # TODO:do transform T1 ^-1 * T2.



    '''
    helper functions
    '''
    def set_status(self, status):
        self.STATUS = String(status.name)


    '''
    def reachTargetPosition(self, target, threshold = 0.1):

        delta_x = math.fabs(self.local_pose.pose.position.x - target.pos_sp[0])
        delta_y = math.fabs(self.local_pose.pose.position.y - target.pos_sp[1])
        delta_z = math.fabs(self.local_pose.pose.position.z - target.pos_sp[2])

        distance = (delta_x + delta_y + delta_z)

        print("distance: ", distance, "threshold: ", threshold)
        if distance < threshold:
            return True
        else:
            return False
'''

    def setMavMode(self, msg):
        pass


if __name__ == '__main__':
    nav = Navigator()

    #FLU meters.
    nav.set_target_postion((10, 0, 2.5))
    nav.keep_navigating()











