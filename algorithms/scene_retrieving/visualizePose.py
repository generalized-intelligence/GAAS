import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# given a csv where each line represents current positon separated by comma, plot position history

slam_pose_path = "./loadedpose.csv"
slam_pose_history = []
slam_pose = open(slam_pose_path).readlines()

for line in slam_pose:
    pose = (float(line.split(',')[0]), float(line.split(',')[1]), float(line.split(',')[2].split('\n')[0]))
    slam_pose_history.append(pose)

slam_pose_x = []
slam_pose_y = []
slam_pose_z = []
for pose in slam_pose_history:
    slam_pose_x.append(pose[0])
    slam_pose_y.append(pose[1])
    slam_pose_z.append(pose[2])

Fig = plt.figure(1)
ax = Axes3D(Fig)
ax.plot(slam_pose_x,slam_pose_y,slam_pose_z,color='r')
plt.show()
