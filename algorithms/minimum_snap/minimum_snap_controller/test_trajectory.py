import numpy as np
from trajectory import testcase_trajectory
from minimum_snap_controller import controller, param, K_param
import pyquaternion as pyq
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def draw_3d(xs, ys, zs, color='b'):
    ax.scatter(xs, ys, zs, c=color, marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

if __name__ == '__main__':


    current_state = [0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0]

    xs=[]
    ys=[]
    zs=[]
    xds=[]
    yds=[]
    zds=[]
    model_param = param()
    K = K_param()
    current_q = pyq.Quaternion(1, 0, 0, 0)
    step = 0.1
    for i in range(200):
        t = i/10.0
        desired_state = testcase_trajectory(t)
        # print "desired_state: ", desired_state
        # omega is used for current drone state, while yaw and yaw_dot is used for desired state
        #         0  1  2  3   4   5   6    7    8    9     10    11    12       13       14       15   16
        # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', omega.x, omega.y, omega.z, yaw, yaw_dot]
        # only position, velocity and angular speed is required from drone current state

        _, _, _, _, ad = controller(current_state, desired_state, model_param, K, current_q)

        v_x = current_state[3] + ad[0] * step
        v_y = current_state[4] + ad[1] * step
        v_z = current_state[5] + ad[2] * step
        x = current_state[0] + v_x * step + 0.5 * ad[0] * step * step
        y = current_state[1] + v_y * step + 0.5 * ad[1] * step * step
        z = current_state[2] + v_z * step + 0.5 * ad[2] * step * step
        a_x = ad[0]
        a_y = ad[1]
        a_z = ad[2]
        j_x = 0
        j_y = 0
        j_z = 0
        w_x = 0
        w_y = 0
        w_z = 0
        yaw = 0
        yaw_dot = 0

        current_state = [x, y, z,
                         v_x, v_y, v_z,
                         a_x, a_y, a_z,
                         j_x, j_y, j_z,
                         w_x, w_y, w_z,
                         yaw, yaw_dot]

        print "Desired accel: ", ad
        # print current_state

        # print x, y, z
        xs.append(x)
        ys.append(y)
        zs.append(z)
        xds.append(desired_state[0])
        yds.append(desired_state[1])
        zds.append(desired_state[2])
        # print ad

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    draw_3d(xs, ys, zs, 'b')
    draw_3d(xds, yds, zds, 'r')
    plt.show()



