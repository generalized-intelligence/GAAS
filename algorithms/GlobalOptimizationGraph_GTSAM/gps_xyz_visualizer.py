#encoding=utf-8
#可视化观察gps和SLAM变化规律.

import rospy
#from sensor_msgs import from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from time import sleep
import pyquaternion
import numpy as np
import math

if __name__ == '__main__':
    rospy.init_node('gog_debug_node')
    odom_pub = rospy.Publisher('/gog_vis_debug', Odometry, queue_size=5)
    lines = None
    with open('/tmp/GlobalOptimizationGraph_main.INFO','r') as f:
        lines = f.readlines()
    #print lines
    i = 0
    #SLAM输入的显示.
    gxl,gyl,gzl ,sxl,syl,szl = [],[],[],[],[],[]
    #mode_slam = False#True#False
    #mode_opti = True#False#True
    #mat = np.matrix([[0,0,1],[1,0,0],[0,1,0]])  # ygz 旋转阵右乘这个,平移不动.
    for l__ in lines:
        if(l__.find('GPS_MEASUREMENT_DEBUG:dxdydh:')>=0):
            gps_dxyz = map(float,l__.split('GPS_MEASUREMENT_DEBUG:dxdydh:')[1].split(';SLAM:')[0].split(','))
            slam_dxyz = map(float,l__.split('GPS_MEASUREMENT_DEBUG:dxdydh:')[1].split(';SLAM:')[1].split(','))
            gx,gy,gz = gps_dxyz
            sx,sy,sz = slam_dxyz
            print gx,gy,gz
            #matched = True
            gxl.append(gx)
            gyl.append(gy)
            gzl.append(gz)
            sxl.append(sx)
            syl.append(sy)
            szl.append(sz)
    import matplotlib.pyplot as plt
    #step<1> 显示r p y三个角度的曲线图.
    ax = plt.plot(gxl,color='#FF8888',linewidth = 3)
    ax = plt.plot(sxl,color='#FF0000',linewidth = 1) 

    ax = plt.plot(gyl,color='#88FF88',linewidth=3)
    ax = plt.plot(syl,color='#00FF00',linewidth=1)

    ax = plt.plot(gzl,color='#8888FF',linewidth=3)
    ax = plt.plot(szl,color='#0000FF',linewidth=1)


    plt.show()



