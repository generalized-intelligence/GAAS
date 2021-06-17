#encoding=utf-8
import astar_config
import heapq
#from simulator.visualization.config.aircraft_config import aircraft_obj

def gen_aircraft_obj():
    ret_val = []

    # for i in range(-12, 12):
    #     for j in range(-12, 12):
    #         ret_val.append((i, j, 0))
    #
    # ret_val.append((0, 0, -4))

    K = 5
    for i in range(-K, K):
        for j in range(-K, K):
            for k in range(-K, K):
                ret_val.append((i, j, k))

    ret_val.append((0, 0, -3))

    # for i in range(-4,4):
    #     for j in range(-4,4):
    #         ret_val.append((i,j,0))
    # ret_val.append((0, 0, -3))

    # for i in range(-2, 2):
    #     for j in range(-2,2):
    #          ret_val.append((i,j,0))First element of openlist
    #ret_val.append((0,0,-2))

    return ret_val

aircraft_obj = {
    "aircraft_points": gen_aircraft_obj(),
    "init_center": (0, 0, 0)
}


#节点类
class Node(object):
    def __init__(self,point):
        self.point = point
        self.parent = None
        self.G = 0
        self.H = 0
    def get_point(self):
        return self.point
    def move_cost(self,p):
        return 1.0 if self.point[2] == p[2] else astar_config.astar_config['z_move_cost']
    def __str__(self):
        return "point : %s ,  G: %.1f,  H: %.1f,  F: %.1f" % (str(self.point),self.G,self.H,self.G+self.H) 



#end_pos          : 结束点(x,y,z)
#open           : 开始列表
#close          : 关闭列表
#is_valid       : 判断点是否允许通过  
#movement_list  : 允许转向的方向
#func_h         : 计算H值的算法
class A_star(object):
    def __init__(self,end_pos):
        self.end_pos = end_pos      
        
        self.openlist = []          
        self.closed = []           
        self.movement_list = astar_config.astar_config['movement_list']     
        self.func_h = astar_config.astar_config['func_h']   
        self.horiontal_radius, self.vertical_radius = self.__aircraft_radius(aircraft_obj['aircraft_points'])
        #print ('aircraft_obj',aircraft_obj)


    def __distance(self,pt1,pt2):
        return ((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2+(pt1[2]-pt2[2])**2)**0.5

    #
    # def __aircraft_radius(self,aircraft_points):
    #     horiontal_radius = 64
    #     vertical_radius = 64
    #     center_point = aircraft_points[0]
    #
    #     for point in aircraft_points:
    #         if point[2]==0:
    #             r = self.__distance(point,center_point)
    #             if horiontal_radius < r:
    #                 horiontal_radius = r
    #         if point[2]<0:
    #             r = self.__distance(point,center_point)
    #             if vertical_radius < r:
    #                 vertical_radius = r
    #
    #     return horiontal_radius, vertical_radius

    def __aircraft_radius(self, aircraft_points):
        horiontal_radius = 0
        vertical_radius = 0

        #center_point = aircraft_points[0]
        center_point = (0, 0, 0)

        print("center point is: ", center_point)

        for point in aircraft_points:
            if point[2]==0:
                r = self.__distance(point,center_point)
                if horiontal_radius < r:
                    horiontal_radius = r
            if point[2]<0:
                r = self.__distance(point,center_point)
                if vertical_radius < r:
                    vertical_radius = r

        print("horizontal radius and vertical: ", horiontal_radius, vertical_radius)
        return horiontal_radius, vertical_radius


    def __success(self, current_obj, center_point_can_go):
        # if reached target point, return True
        if center_point_can_go:
            if current_obj.get_point() == self.end_pos:
                return True
        # if distance small than body radius, return True
        elif not center_point_can_go:
            if self.__distance(current_obj.get_point(), self.end_pos)<=(max(self.horiontal_radius, self.vertical_radius)):
                return True

        return False


    def debug_get_openlist_info(self):
        first_element = heapq.heappop(self.openlist)
        node = first_element[1]
        #print ('First element of openlist:',node.get_point())
        heapq.heappush(self.openlist, ((node.G+node.H), node))



    def find_path(self, start_pos, obstacle_pos_list):
        self.openlist = []  
        self.closed = set()
        start_obj = Node(start_pos)
        #self.openlist.append(start_obj)

        heapq.heappush(self.openlist, ((start_obj.G+start_obj.H), start_obj))
        center_point_can_go = self.is_valid(self.end_pos, obstacle_pos_list)   #判断中心点能否到达，如果中心点不能到达，则判断机身能否到达
        path = []
        count = 0

        if self.end_pos in obstacle_pos_list:
            print('Target Position Found inside Obstacle, Finding Path Failed!')
            return None

        while self.openlist:
            #print ('obstacle_pos_list',obstacle_pos_list)
            self.debug_get_openlist_info()
            count += 1

            current_obj = heapq.heappop(self.openlist)[1]

            if self.__success(current_obj, center_point_can_go):   #判断是否到达

                while current_obj.parent:
                    path.append(current_obj.get_point())
                    current_obj = current_obj.parent

                path.append(current_obj.get_point())

                print('count:', count)
                print("extend:", len(self.openlist)+len(self.closed))

                return path[::-1]


            self.closed.add(current_obj)

            self.extend_round(current_obj, obstacle_pos_list)

        print('Open List Run Out, No Path Found.')


    #遍历周围节点，检测碰撞，寻找路径        
    def extend_round(self, current_obj, obstacle_pos_list):

        #6 direction
        for x, y, z in self.movement_list:

            new_point = (x+current_obj.get_point()[0],
                         y+current_obj.get_point()[1],
                         z+current_obj.get_point()[2])

            node_obj = Node(new_point)

            if not self.is_valid(new_point, obstacle_pos_list):
                continue

            if self.is_in_closedlist(new_point):
                continue

            if self.is_in_openlist(new_point):
                new_g = current_obj.G + current_obj.move_cost(new_point)
                if node_obj.G > new_g:
                    node_obj.G = new_g
                    node_obj.parent = current_obj
            else:
                node_obj = Node(new_point)
                node_obj.G = current_obj.G + current_obj.move_cost(new_point)
                node_obj.H = self.func_h(node_obj.get_point(), self.end_pos)
                node_obj.parent = current_obj
                #self.openlist.append(node_obj)
                heapq.heappush(self.openlist, ((node_obj.G+node_obj.H), node_obj))

        
    def is_in_closedlist(self, p):
        for node in self.closed:
            if node.point[0] == p[0] and node.point[1] == p[1] and node.point[2] == p[2]:
                return True
        return False

    def is_in_openlist(self,p):
        for heap_obj in self.openlist:
            node = heap_obj[1]
            if node.point[0]==p[0] and node.point[1]==p[1] and node.point[2]==p[2]:
                return True
        return False


    def is_valid(self, pt, obstacle_map):
        global aircraft_obj
        obstacle_map_indexed = set(obstacle_map)
        aircraft_points = []

        for item in aircraft_obj['aircraft_points']:
            aircraft_points.append(
                (item[0]+pt[0],
                 item[1]+pt[1],
                 item[2]+pt[2])
            )
            if item[2]+pt[2] <= 0:
                return False
        
        

        aircraft_indexed = set(aircraft_points)
        #import pdb;pdb.set_trace()
        if obstacle_map_indexed & aircraft_indexed:
            return False
        else:
            return True


    def path_is_valid(self,path_points,obstacle_map):
        #if set(map(lambda x:(x[0],x[1],x[2]),path_points)) & set(obstacle_map):
        #    return False
        for pt in path_points:
            if not self.is_valid(pt, obstacle_map):
                print('Path is invalid.')
                return False


        print('Path is valid.')

        #print 'path_points:',path_points
        #print 'obstacle map:',obstacle_map
        return True


        # if pt[2]<1:
        #     return False
        # for obstacle_point in obstacle_map:
        #     if obstacle_point[2]<pt[2]:
        #         if self.__distance(pt,obstacle_point)<=self.vertical_radius:
        #             return False
        #     else:
        #         if self.__distance(pt,obstacle_point)<=self.horiontal_radius:
        #             return False
        # return True

