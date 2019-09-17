#encoding=utf-8

#日志里的内容用rviz显示出来.
import rospy
#from sensor_msgs import from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from time import sleep




if __name__ == '__main__':
    rospy.init_node('gog_debug_node')
    odom_pub = rospy.Publisher('/gog_vis_debug', Odometry, queue_size=5)


    lines = None
    with open('/tmp/GlobalOptimizationGraph_main.INFO','r') as f:
        lines = f.readlines()
    #print lines
    i = 0
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
            sleep(0.01)
            odom_pub.publish(o)








