#encoding=utf-8

#日志里的内容用rviz显示出来.
import rospy
#from sensor_msgs import from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from time import sleep
import pyquaternion
import numpy as np


def process_pose_with_mat(mat,px,py,pz,x,y,z,w):
    print mat,px,py,pz,x,y,z,w
    print np.array([px,py,pz]).T
    outp = (np.matrix([[px],[py],[pz]]))
    quat = pyquaternion.Quaternion(w=w,x=x,y=y,z=z)
    out_rotation_mat = quat.rotation_matrix*mat#mat*quat.rotation_matrix
    out_q = pyquaternion.Quaternion(matrix=out_rotation_mat)
    return (outp[0],outp[1],outp[2]),(out_q.x,out_q.y,out_q.z,out_q.w)

def process_quat()

if __name__ == '__main__':
    rospy.init_node('gog_debug_node')
    odom_pub = rospy.Publisher('/gog_vis_debug', Odometry, queue_size=5)
    lines = None
    with open('/tmp/GlobalOptimizationGraph_main.INFO','r') as f:
        lines = f.readlines()
    #print lines
    i = 0
    #优化后结果.
    '''
    for l__ in lines:
        if(l__.find('full stat')>=0):
            #print l__
            orient = map(float,l__.split('|')[1].split(','))
            pos = map(float,l__.split('|')[2].split(','))
            print 'pos:',pos,'orient:',orient
            o = Odometry()
            o.header.frame_id = '/map'
            #o.id = i
            #i+=1
            o.pose.pose.orientation.x = orient[0]
            o.pose.pose.orientation.y = orient[1]
            o.pose.pose.orientation.z = orient[2]
            o.pose.pose.orientation.w = orient[3]
            o.pose.pose.position.x = pos[0]
            o.pose.pose.position.y = pos[1]
            o.pose.pose.position.z = pos[2]
            #sleep(0.01)
            sleep(0.05)
            odom_pub.publish(o)

    '''
    #SLAM输入的显示.
    mat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    #mat = np.matrix([[0,0,1],[1,0,0],[0,1,0]])  # ygz 旋转阵右乘这个,平移不动.
    for l__ in lines:
        if(l__.find('Full info of slam input:')>0):
            px,py,pz = map(float,l__.split("input:")[1].split(";")[0].split(','))
            x,y,z,w = map(float,l__.split("input:")[1].split(";")[1].split(','))
            (px,py,pz),(x,y,z,w) = process_pose_with_mat(mat,px,py,pz,x,y,z,w)
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
            #sleep(0.01)
            sleep(0.05)
            odom_pub.publish(o)
           
            







