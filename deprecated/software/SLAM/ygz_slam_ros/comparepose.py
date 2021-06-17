import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


slam_pose_path = "/home/gishr/software/GAAS_dev/software/SLAM/ygz_slam_ros/slampose200.csv"
px4_pose_path = "/home/gishr/software/GAAS_dev/software/SLAM/ygz_slam_ros/slampose20.csv"

slam_pose_history = []
px4_pose_history = []

slam_pose = open(slam_pose_path).readlines()
px4_pose = open(px4_pose_path).readlines()

for line in slam_pose:
    pose = (float(line.split(',')[0]), float(line.split(',')[1]), float(line.split(',')[2].split('\n')[0]))
    slam_pose_history.append(pose)


for line in px4_pose:
    pose = (float(line.split(',')[0]), float(line.split(',')[1]), float(line.split(',')[2].split('\n')[0]))
    px4_pose_history.append(pose)


print(slam_pose_history[0])

print(px4_pose_history[0])

slam_pose_x = []
slam_pose_y = []
slam_pose_z = []
for pose in slam_pose_history:
    slam_pose_x.append(pose[0])
    slam_pose_y.append(pose[1])
    slam_pose_z.append(pose[2])

px4_pose_x=[]
px4_pose_y=[]
px4_pose_z=[]
for pose in px4_pose_history:
    px4_pose_x.append(pose[0])
    px4_pose_y.append(pose[1])
    px4_pose_z.append(pose[2])

Fig = plt.figure(1)
ax = Axes3D(Fig)
ax.plot(slam_pose_x,slam_pose_y,slam_pose_z,color='r')
ax.plot(px4_pose_x,px4_pose_y,px4_pose_z,color='b')
plt.show()
