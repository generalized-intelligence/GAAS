#encoding=utf-8
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

#check_file='without_gps_log.log'

def get_gps_position_by_lines(line):
    if line.find('[GPS_INFO]')>=0 and line.find("GPS_relative_pos")>0:
        print line
        print line.split(':')[-1].split(',')
        return map(float,line.split(':')[-1].split(',')[-4:])
    return None
def get_checkfile(check_file_name):
    return open(check_file_name).readlines()

def get_check_yaw_data(check_file):
    ys = []
    ps = []
    rs = []
    for line in check_file:
      if line.find('get_yaw_from_slam_msg')>0:
        print line
        yaw = float(line.split('get_yaw_from_slam_msg:')[1].split('\t')[0])
        print float(line.split('pitch:')[1].split('\t')[0])
        pitch = float(line.split('pitch:')[1].split('\t')[0] )
         
        print  float(line.split('roll:')[1])
        roll = float(line.split('roll:')[1])
        ys.append(yaw)
        ps.append(pitch)
        rs.append(roll)
        
    return ys,ps,rs


    

check_file_with_gps = get_checkfile('/tmp/GlobalOptimizationGraph_main.INFO') # read gps and visualize.


#xs,ys,zs,ts = get_check_gps_data(check_file_with_gps)
ys ,ps, rs = get_check_yaw_data(check_file_with_gps)

ax = plt.plot(ys,'r',linewidth = 3)
ax = plt.plot(ps,'y',linewidth=3)
ax = plt.plot(rs,'b',linewidth=3)
#x, y, z = data[0], data[1], data[2]

#ax.scatter(ys,c='y')  # gps.
#ax.scatter(yslam,zslam,xslam,c = 'b')
#ax.scatter(xslam,yslam,zslam,c='b')
#ax.scatter(x_opt,y_opt,z_opt,c='r')
# 绘制数据点 
#ax.set_xlabel('X') 
#ax.set_ylabel('Y') 
#ax.set_zlabel('Z') # 坐标轴 
#ax.set_xlim3d(-20,20)
#ax.set_ylim3d(-20,20)
#ax.set_zlim3d(-20,20)
plt.show()

plt.scatter(xs,ys,alpha=0.6)  # 绘制散点图，透明度为0.6（这样颜色浅一点，比较好看）
plt.scatter(xslam,yslam,alpha = 0.5)
plt.figure()




plt.plot(xs, 'r')
plt.plot(ys, 'y')
plt.plot(zs, 'b')
plt.plot(xsgps,'r.')
plt.plot(ysgps,'y.')
plt.plot(zsgps,'b.')

#plt.plot(xs_gt_al,'r.')
#plt.plot(ys_gt_al,'y.')
#plt.plot(zs_gt_al,'b.')
plt.show()


