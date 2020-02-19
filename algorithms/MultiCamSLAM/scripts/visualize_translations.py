#encoding=utf-8

import pdb
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 

def parse_line_pose(line):
    if line.find('optimized translation:')>=0:
        #进行相应的操作.
        #node_id = int(line.split('[OFFLINE OPTIMIZED RESULT] node_id:')[1].split(';xyz:')[0])
        node_xyz = map(float, line.split('optimized translation:')[1].split(','))
        x,y,z = node_xyz
        #return node_id,x,y,z
        return x,y,z
    else:
        return None


def main():
    print('usage: python visualize_poses.py')
    log_path = '/tmp/test_SLAM_offline.INFO'
    xs,ys,zs = {},{},{}
    xl,yl,zl = [],[],[]
    xl_estimated,yl_estimated,zl_estimated = [],[],[]


    lm_x,lm_y,lm_z = [],[],[]
    with open(log_path,'rb') as f:
        lines = f.readlines()
        processed_lines = filter(lambda x:x is not None ,map(parse_line_pose,lines))
        #processed_lines_landmark = filter(lambda x:x is not None, map(parse_line_landmark,lines))
        #processed_lines_before_optimize = filter(lambda x:x is not None ,map(parse_line_pose_prev_optimize,lines))

        #pdb.set_trace()
        for pl in processed_lines:
            xl.append(pl[0])
            yl.append(pl[1])
            zl.append(pl[2])
    print ('pts size:',len(xl))
    ax = plt.subplot(111, projection='3d') # 创建一个三维的绘图工程 
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z') # 坐标轴 
    ax.set_xlim3d(-40,40)
    ax.set_ylim3d(-40,40)
    ax.set_zlim3d(-40,40)

    ax.scatter(xl, yl, zl, c='y')  # optimized result
    #ax.scatter(lm_x,lm_y,lm_z,c = 'b') # optmized landmarks (optional)
    #ax.scatter(xl_estimated,yl_estimated,zl_estimated,c='r')
    plt.show()
    ax = plt.plot(xl,'r',linewidth = 3)
    ax = plt.plot(yl,'y',linewidth=3)
    ax = plt.plot(zl,'b',linewidth=3)
#    ax = plt.plot(xl_estimated,'r',linewidth = 1)
#    ax = plt.plot(yl_estimated,'y',linewidth = 1)
#    ax = plt.plot(zl_estimated,'b',linewidth = 1)

    plt.show()
    

if __name__ == '__main__':
    main()
