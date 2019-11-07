import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re


def readFile(path, z_offset=0):
    x = []
    y = []
    z = []
    thres = 1e2
    slam_file = open(path).readlines()
    for line in slam_file:
        line = re.split(',|\n', line)[0:-1]
        if(abs(float(line[0]))>thres or abs(float(line[1]))>thres or abs(float(line[2]))>thres):
            continue

        x.append(float(line[0]))
        y.append(float(line[1]))
        z.append(float(line[2]) + z_offset)
        print line
    return x, y, z


def putIdx(path, ax, z_offset=1):
    x = []
    y = []
    z = []
    thres = 1e2
    slam_file = open(path).readlines()
    for idx, line in enumerate(slam_file):
        
        if (idx % 40 != 0):
            continue

        line = re.split(',|\n', line)[0:-1]
        if(abs(float(line[0]))>thres or abs(float(line[1]))>thres or abs(float(line[2]))>thres):
            continue

        ax.text(float(line[0]), float(line[1]), float(line[2]) + z_offset, str(idx), color='red')


def putFlag(path, ax, z_offset=1):
    x = []
    y = []
    z = []
    thres = 1e2
    slam_file = open(path).readlines()
    for idx, line in enumerate(slam_file):
        
        if (idx % 10 != 0):
            continue

        line = re.split(',|\n', line)[0:-1]
        if(abs(float(line[0]))>thres or abs(float(line[1]))>thres or abs(float(line[2]))>thres):
            continue

        ax.text(float(line[0]), float(line[1]), float(line[2]) + z_offset, str(int(line[3])), color='red')


if __name__ == '__main__':

    slam_x, slam_y, slam_z = readFile("slam_position.txt")
    gps_x, gps_y, gps_z = readFile("gps_position.txt", 1)
    result_x, result_y, result_z = readFile("fused_position.txt", 2)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(slam_x, slam_y, slam_z, c='r', marker='o')
    ax.scatter(gps_x, gps_y, gps_z, c='g', marker='v')
    ax.scatter(result_x, result_y, result_z, c='b', marker='^')
    
    putFlag("slam_position.txt", ax, z_offset=1)
    #putIdx("fused_position.txt", ax, 1)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
