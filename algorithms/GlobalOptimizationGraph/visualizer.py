#encoding=utf-8
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

#check_file='without_gps_log.log'

RANGE=40
def get_gps_position_by_lines(line):
    if line.find('[GPS_INFO]')>=0 and line.find("GPS_relative_pos")>0:
        print line
        print line.split(':')[-1].split(',')
        return map(float,line.split(':')[-1].split(',')[-4:])
    return None

def get_slam_position_by_lines(line):
    if line.find('[SLAM_INFO]')>=0  and line.find("slam position")>0:#and line.find("slam_orig")>0:#        print line
        print line.split(':')[-1].split(',')
        return map(float,line.split(':')[-1].split(',')[-4:])
    return None

def get_optimized_position_by_lines(line):
    if line.find('[OPTIMIZER_INFO]')>=0 and line.find("Last_pos_of_optimizer")>=0:
        print line
        print line.split(':')[-1].split(',')
        return map(float,line.split(':')[-1].split(',')[-4:])
    return None


def get_checkfile(check_file_name):
    return open(check_file_name).readlines()

def get_check_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = get_gps_position_by_lines(line)
        if res:
            xs.append(res[0])
            ys.append(res[1])
            zs.append(res[2])
            #ts.append(res[3])
    return xs,ys,zs#,ts

def get_check_slam_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = get_slam_position_by_lines(line)
        if res:
            xs.append(res[0])
            ys.append(res[1])
            zs.append(res[2])
            #ts.append(res[3])
    return xs,ys,zs#,ts



def get_check_gps_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = map(float,line.split(','))
        xs.append(res[1])
        ys.append(res[2])
        zs.append(res[3])
        ts.append(res[0])
    return xs,ys,zs,ts


def get_check_optimized_pos_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = get_optimized_position_by_lines(line)
        if res:
            xs.append(res[0])
            ys.append(res[1])
            zs.append(res[2])
    return xs,ys,zs


#check_file_without_gps = get_checkfile('standard.log')
    

check_file_with_gps = get_checkfile('./log.log') # read gps and visualize.


#xs,ys,zs,ts = get_check_gps_data(check_file_with_gps)
xs,ys,zs = get_check_data(check_file_with_gps)

xslam,yslam,zslam = get_check_slam_data(check_file_with_gps)
x_opt,y_opt,z_opt = get_check_optimized_pos_data(check_file_with_gps)
#x, y, z = data[0], data[1], data[2]
print ('len x_gps:',len(xs),'len x_slam:',len(xslam),'len x_opt:',len(x_opt))
ax = plt.subplot(111, projection='3d') # 创建一个三维的绘图工程 


ax.scatter(xs, ys, zs, c='y',s = 10) 
#ax.scatter(yslam,zslam,xslam,c = 'b')
ax.scatter(xslam,yslam,zslam,c='b',alpha=0.5,s = 1)
ax.scatter(x_opt,y_opt,z_opt,c='r',alpha=0.5,s = 1)
# 绘制数据点 
ax.set_xlabel('X') 
ax.set_ylabel('Y') 
ax.set_zlabel('Z') # 坐标轴 
ax.set_xlim3d(-1*RANGE,RANGE)
ax.set_ylim3d(-1*RANGE,RANGE)
ax.set_zlim3d(-1*RANGE,RANGE)
plt.show()
'''
plt.scatter(xs,ys,alpha=0.6)  # 绘制散点图，透明度为0.6（这样颜色浅一点，比较好看）
plt.scatter(xslam,yslam,alpha = 0.5)
plt.figure()




plt.plot(xs, 'r')
plt.plot(ys, 'y')
plt.plot(zs, 'b')
plt.plot(xsgps,'r.')
plt.plot(ysgps,'y.')
plt.plot(zsgps,'b.')
'''
#plt.plot(xs_gt_al,'r.')
#plt.plot(ys_gt_al,'y.')
#plt.plot(zs_gt_al,'b.')
#plt.show()


