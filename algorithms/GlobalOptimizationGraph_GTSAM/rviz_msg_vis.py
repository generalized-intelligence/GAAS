#encoding=utf-8

#日志里的内容用rviz显示出来.
import rospy
#from sensor_msgs import from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from time import sleep
import pyquaternion
import numpy as np
import math

def process_pose_with_mat(mat,px,py,pz,x,y,z,w):
    print mat,px,py,pz,x,y,z,w
    print np.array([px,py,pz]).T
    outp = mat*(np.matrix([[px],[py],[pz]]))
    quat = pyquaternion.Quaternion(w=w,x=x,y=y,z=z)
    #out_rotation_mat = quat.rotation_matrix*mat
    out_rotation_mat = mat*quat.rotation_matrix
    out_q = pyquaternion.Quaternion(matrix=out_rotation_mat)
    return (outp[0],outp[1],outp[2]),(out_q.x,out_q.y,out_q.z,out_q.w)

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
    return r,p,y


if __name__ == '__main__':
    rospy.init_node('gog_debug_node')
    odom_pub = rospy.Publisher('/gog_vis_debug', Odometry, queue_size=5)
    lines = None
    with open('/tmp/GlobalOptimizationGraph_main.INFO','r') as f:
        lines = f.readlines()
    #print lines
    i = 0
    #优化后结果.
    #SLAM输入的显示.
    mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    #mat = np.matrix([[0,0,1],[1,0,0],[0,1,0]])
    #mat = np.matrix([[0,1,0],[0,0,1],[1,0,0]])
    rs,ps,ys,px_list,py_list,pz_list = [],[],[],[],[],[]
    mode_slam =  False#True#False
    mode_opti = True#False#True
    #mat = np.matrix([[0,0,1],[1,0,0],[0,1,0]])  # ygz 旋转阵右乘这个,平移不动.
    for l__ in lines:
      px,py,pz = 0,0,0
      x,y,z,w = 0,0,0,0
      matched = False
      if(mode_slam):
        if(l__.find('Full info of slam input:')>0):
            px,py,pz = map(float,l__.split("input:")[1].split(";")[0].split(','))
            x,y,z,w = map(float,l__.split("input:")[1].split(";")[1].split(','))
            #(px,py,pz),(x,y,z,w) = process_pose_with_mat(mat,px,py,pz,x,y,z,w)
            matched = True
      elif(mode_opti):
        if(l__.find('Current full status:')>=0):
            orient = map(float,l__.split('|')[1].split(','))
            pos = map(float,l__.split('|')[2].split(','))
            px,py,pz = pos
            x,y,z,w = orient
            #(px,py,pz),(x,y,z,w) = process_pose_with_mat(mat,px,py,pz,x,y,z,w)
            matched = True
      if(matched):
        o = Odometry()
        o.header.frame_id = '/map'
        #o.id = i
        #i+=1
        o.pose.pose.orientation.x = x
        o.pose.pose.orientation.y = y
        o.pose.pose.orientation.z = z
        o.pose.pose.orientation.w = w
        o.pose.pose.position.x = px
        o.pose.pose.position.y = py
        o.pose.pose.position.z = pz
        R_,P_,Y_ = get_rpy_from_quat(x,y,z,w)
        rs.append(R_)
        ps.append(P_)
        ys.append(Y_)
        px_list.append(px)
        py_list.append(py)
        pz_list.append(pz)
        #sleep(0.001)
        #sleep(0.01)
        #odom_pub.publish(o)

    import matplotlib.pyplot as plt
    #step<1> 显示r p y三个角度的曲线图.
    ax = plt.plot(ys,'r',linewidth = 3)
    ax = plt.plot(ps,'y',linewidth=3)
    ax = plt.plot(rs,'b',linewidth=3)
    plt.show()

    #step<2> 显示x,y位置关系曲线图.
    ax = plt.plot(px_list,'r',linewidth = 1)
    ax = plt.plot(py_list,'y',linewidth = 1)
    ax = plt.plot(pz_list,'b',linewidth = 1)
    plt.show()
            







