#encoding=utf-8
from astar import A_star
from multiprocessing import Queue,Process
from time import sleep
import rospy


class Driver:
    def __init__(self): #init_pos, end_pos,command_queue,returnval_queue):
        #self.algo = A_star(end_pos)
        #self.init_pos = init_pos
        #self.current_pos = init_pos
        #self.end_pos = end_pos
        #self.command_queue = command_queue
        #self.returnval_queue = returnval_queue
        #self.sleeptime = 0.1  #TODO:delete this.
        self.TIME_DELAY_THRESHOLD = 500 # TODO: fix this.

        #to keep a obstacle set buffer.
        self.obstacle_set = set()
        self.obs_set_last_update_time = rospy.rostime.get_time()

    def set_obstacle_set(self,obstacle_set):
        self.obstacle_set = obstacle_set
        self.obs_set_last_update_time = rospy.rostime.get_time()

    def get_obstacles_around(self):
        current_time = rospy.rostime.get_time()
        if current_time - self.obs_set_last_update_time > self.TIME_DELAY_THRESHOLD:
            print ("Warning:Buffer timeout!Delay:",current_time-self.obs_set_last_update_time)
        return self.obstacle_set

    def move_aircraft(self,vec):
        while not self.command_queue.empty():
            sleep(self.sleeptime)
        self.command_queue.put({'type':'move','movement':vec})
        while self.returnval_queue.empty():
            sleep(self.sleeptime)
        result = self.returnval_queue.get()
        return result
    '''
    def drive(self): 
        relative_pos=(0,0,0)
        while self.current_pos!=self.end_pos:           # Till task is finished:
            self.algo = A_star(self.end_pos)
            print 'Move 1 step'
            obstacle_map=self.get_obstacles_around()  # TODO:加入障碍记忆.
            print 'From ',self.current_pos
            for item in obstacle_map:
                self.obstacle_set.add(item)
            import time
            t1 = time.time()
            path = self.algo.find_path(self.current_pos,list(self.obstacle_set))
            t2 = time.time()
            print 'A* time cost:' ,(t2-t1)
            if not path:
                return
            print 'path:',path
            for next_move in path:                      #Till path has run out to a invalid point or finished.
                print 'current_pos:',self.current_pos
                next_pos = next_move
                relative_pos = (next_pos[0]-self.current_pos[0],next_pos[1]-self.current_pos[1],next_pos[2]-self.current_pos[2])
                print 'next_move : ',next_move
                print "relativ_move : ",relative_pos
                if not self.algo.is_valid(next_pos,list(self.obstacle_set)):
                    break
                self.current_pos = next_pos
                print 'ob_set:',list(self.obstacle_set)
                
                self.move_aircraft(relative_pos)
                sleep(0.5)
                obstacle_map=self.get_obstacles_around()
                for item in obstacle_map:
                    self.obstacle_set.add(item)
                predict_move = (self.current_pos[0]+relative_pos[0],self.current_pos[1]+relative_pos[1],self.current_pos[2]+relative_pos[2])
                print 'ob_map : ',obstacle_map
                print "prdict_move : ",predict_move
                if not self.algo.is_valid(predict_move,list(self.obstacle_set)):
                    print 'cant goooooo'
                    break
'''


