import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re






def readfile(path):
    x = []
    y = []
    z = []
    slam_file = open(path).readlines()
    for line in slam_file:
        line = re.split(',|\n', line)[0:-1]
        x.append(float(line[0]))
        y.append(float(line[1]))
        z.append(float(line[2]))
        print line
    return x, y, z




if __name__ == '__main__':

    slam_x, slam_y, slam_z = readfile("slam_position.txt")

    gps_x, gps_y, gps_z = readfile("gps_position.txt")

    result_x, result_y, result_z = readfile("fused_position.txt")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(slam_x, slam_y, slam_z, c='r', marker='o')
    ax.scatter(gps_x, gps_y, gps_z, c='g', marker='v')
    ax.scatter(result_x, result_y, result_z, c='b', marker='^')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
