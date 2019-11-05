#encoding=utf-8

#日志里的内容用rviz显示出来.
import rospy
from nav_msgs.msg import Odometry
from time import sleep
import pyquaternion
import numpy as np
import math


def get_rpy_from_mat(mat):
    q = pyquaternion.Quaternion(matrix = mat)
    return get_rpy_from_quat(q.x,q.y,q.z,q.w)

def get_rpy_from_quat(x,y,z,w):
    quat = pyquaternion.Quaternion(x=x,y=y,z=z,w=w)
    R = quat.rotation_matrix
    n = R[:,0]
    o = R[:,1]
    a = R[:,2]
    #print "R:",R,"n:",n
    #Eigen::Vector3d o = R.col(1);
    #Eigen::Vector3d a = R.col(2);

    y = math.atan2(n[1], n[0]);
    p = math.atan2(-1*n[2], n[0] * math.cos(y) + n[1] * math.sin(y));
    r = math.atan2(a[0] * math.sin(y) - a[1] * math.cos(y), -1*o[0] * math.sin(y) + o[1] * math.cos(y));
    #y+=3.1415926535
    if y>3.1415926535:
        y-= 2*3.1415926535
    return r,p,y
def get_mat_from_rpy(r,p,y):
    #y+=3.1415926535
    mat1 = np.matrix([
			[math.cos(y), -1*math.sin(y), 0],
		      	[math.sin(y), math.cos(y), 0],
      			[0, 0, 1]
		])
    mat2 = np.matrix([
			[math.cos(p), 0., math.sin(p)],
			[0., 1., 0.], 
			[-math.sin(p), 0., math.cos(p)]
		])
    mat3 = np.matrix([
			[1., 0., 0.], 
			[0., math.cos(r), -1*math.sin(r)],
			[0., math.sin(r), math.cos(r)]
		])
    mat_res = mat1*mat2*mat3
    return mat_res

if __name__ == '__main__':
    rospy.init_node('gog_debug_node')
    odom_pub = rospy.Publisher('/gog_vis_debug', Odometry, queue_size=5)
    lines = None
    with open('/tmp/GlobalOptimizationGraph_main.INFO','r') as f:
        lines = f.readlines()
    i = 0
    ys,ps,rs = [],[],[]
    yinner,pinner,rinner = [],[],[]
    for l__ in lines:
      if l__.find('in getnewquat(): input original ypr(by quat) val:')>0:
        Y_,P_,R_ = map(float,l__.split('in getnewquat(): input original ypr(by quat) val:')[1].split(','))
        ys.append(Y_)
        ps.append(P_)
        rs.append(R_)
      elif l__.find('after transformed in getNewQuat() ypr:')>0:
        Y_,P_,R_ = map(float,l__.split('after transformed in getNewQuat() ypr:')[1].split(','))
        yinner.append(Y_)
        pinner.append(P_)
        rinner.append(R_)
    import matplotlib.pyplot as plt
    #step<1> 显示r p y三个角度的曲线图.
    ax = plt.plot(ys,'r',linewidth = 3)
    ax = plt.plot(ps,'y',linewidth=3)
    ax = plt.plot(rs,'b',linewidth=3)
    #细线是内部转换后的YPR,粗线是输入的YPR
    ax = plt.plot(yinner,'r',linewidth = 1)
    ax = plt.plot(pinner,'y',linewidth = 1)
    ax = plt.plot(rinner,'b',linewidth = 1)
    
    plt.show()
            







