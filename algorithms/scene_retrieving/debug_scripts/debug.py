import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

raw_file = open("../all.txt").readlines()

retrieved_x = []
retrieved_y = []
retrieved_z = []

mavros_x = []
mavros_y = []
mavros_z = []

target_x = []
target_y = []
target_z = []

slam_x = []
slam_y = []
slam_z = []

for idx, line in enumerate(raw_file):
    line = re.split(',|\n', line)
    print(line)

    retrieved_x.append(float(line[1]))
    retrieved_y.append(float(line[2]))
    retrieved_z.append(float(line[3]))

    mavros_x.append(float(line[4]))
    mavros_y.append(float(line[5]))
    mavros_z.append(float(line[6]))

    target_x.append(float(line[7]))
    target_y.append(float(line[8]))
    target_z.append(float(line[9]))
	
    deviation_factor = float(line[10])
	
    ax.text(float(line[4]), float(line[5]), float(line[6]), str(idx), color='red')
    ax.text(float(line[1]), float(line[2]), float(line[3]), str(idx), color='green')
    ax.text(float(line[1]), float(line[2]), float(line[3])+5.0, str(deviation_factor), color='green')

ax.scatter3D(mavros_x, mavros_y, mavros_z, c='r')
ax.scatter3D(retrieved_x, retrieved_y, retrieved_z, c='g')
ax.scatter3D(target_x, target_y, target_z, c='b')

plt.show()
