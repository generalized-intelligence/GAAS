import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def testcase_trajectory(t):

    # omega is used for current drone state, while yaw and yaw_dot is used for desired state
    #         0  1  2  3   4   5   6    7    8    9     10    11    12       13       14       15   16
    # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', omega.x, omega.y, omega.z, yaw, yaw_dot]

    t = t
    x = 5*np.sin(t)
    y = t
    z = t
    yaw = 0
    yaw_dot = 0
    x_dot = 5*np.cos(t)
    y_dot = 1
    z_dot = 1
    x_dot_dot = -5*np.sin(t)
    y_dot_dot = 0
    z_dot_dot = 0
    x_dot_dot_dot = -5*np.cos(t)
    y_dot_dot_dot = 0
    z_dot_dot_dot = 0

    return [x, y, z,
            x_dot, y_dot, z_dot,
            x_dot_dot, y_dot_dot, z_dot_dot,
            x_dot_dot_dot, y_dot_dot_dot, z_dot_dot_dot,
            0, 0, 0,
            yaw, yaw_dot]



# create your own trajecoty here



if __name__ == '__main__':

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs = []
    ys = []
    zs = []
    for i in range(300):
        t = i/10.0

        state = testcase_trajectory(t)
        x = state[0]
        y = state[1]
        z = state[2]

        xs.append(x)
        ys.append(y)
        zs.append(z)

    ax.scatter(xs, ys, zs, c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()
