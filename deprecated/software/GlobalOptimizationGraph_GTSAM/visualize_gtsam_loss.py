#encoding=utf-8

#日志里的内容用rviz显示出来.
import rospy
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
    chi2s,dofs = [],[]
    for l__ in lines:
      if l__.find('[Optimizer INFO] Current dof and chi2:')>0:
        dof,chi2 = map(float,l__.split('[Optimizer INFO] Current dof and chi2:')[1].split(','))
        dofs.append(dof)
        chi2s.append(chi2)
    import matplotlib.pyplot as plt
    #step<1> 显示r p y三个角度的曲线图.
    ax = plt.plot(chi2s,'r',linewidth = 1)
    plt.show()
    ax = plt.plot(dofs,'b',linewidth = 3)
    plt.show()
            







