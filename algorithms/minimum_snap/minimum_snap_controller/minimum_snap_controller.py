import numpy as np
import pyquaternion as pyq

def skew_matrix(vec):
    skewed = np.array([[0, -vec[2], vec[1]],
                       [vec[2], 0, -vec[0]],
                       [-vec[1], vec[0], 0]])
    # print("vec: ", vec)
    # print("skewed: ", skewed)
    return skewed

def vee(matrix):
    vec = np.array([-matrix[1, 2], matrix[0, 2], -matrix[0, 1]])
    return vec

class param:
    def __init__(self):
        self.mass = 1.477 * 2 * 3
        self.grav = 9.807

class K_param:
    def __init__(self):
        self.Kp = 10
        self.Kv = 20
        self.Kr = 10
        self.K_omega = 3

def controller(current_state, desired_state, model_param, K, current_q):
    # ref: Minimum Snap Trajectory Generation and Control for Quadrotors by Daniel Mellinger and Vijay Kumar

    # omega is used for current drone state, while yaw and yaw_dot is used for desired state
    #         0  1  2  3   4   5   6    7    8    9     10    11    12       13       14       15   16
    # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', omega.x, omega.y, omega.z, yaw, yaw_dot]
    # only position, velocity and angular speed is required from drone current state

    # position error
    ep = np.array([current_state[0]-desired_state[0],
                   current_state[1]-desired_state[1],
                   current_state[2]-desired_state[2]])

    # velocity error
    ev = np.array([current_state[3]-desired_state[3],
                   current_state[4]-desired_state[4],
                   current_state[5]-desired_state[5]])

    # print("-----------------------------------------------")
    # print("current state: ", current_state)
    # print("desired state: ", desired_state)

    Kp = K.Kp
    Kv = K.Kv
    KR = K.Kr
    K_omega = K.K_omega
    m = model_param.mass
    g = model_param.grav
    zw = np.array([0, 0, 1])

    Fdesired = -Kp*ep - Kv*ev + m*g*zw + m*np.array([desired_state[6],
                                                     desired_state[7],
                                                     desired_state[8]])

    current_quaternion = pyq.Quaternion(current_q.w, current_q.x, current_q.y, current_q.z)
    R = current_quaternion.rotation_matrix
    # R = np.array([[1, 0, 0],
    #               [0, 1, 0],
    #               [0, 0, 1]])
    zb = R[:, 2]

    u1 = np.dot(Fdesired, zb)

    current_acc = -g*zw + u1*zb

    ea = np.array([current_acc[0] - desired_state[6],
                   current_acc[1] - desired_state[7],
                   current_acc[2] - desired_state[8]])

    Fd_dot = -Kp*ev - Kv*ea + m*np.array([desired_state[9],
                                          desired_state[10],
                                          desired_state[11]])

    # print "Fd_dot: ", Fd_dot, -Kp*ev, - Kv*ea, m*np.array([desired_state[9],
    #                                       desired_state[10],
    #                                       desired_state[11]])

    zbd = Fdesired/np.linalg.norm(Fdesired)

    yawd = desired_state[15]

    xcd = np.array([np.cos(yawd), np.sin(yawd), 0])
    ybd = np.dot(skew_matrix(zbd), xcd) / np.linalg.norm(np.dot(skew_matrix(zbd), xcd))
    xbd = np.dot(skew_matrix(ybd), zbd)

    Rd1 = np.array([[xbd[0], ybd[0], zbd[0]],
                   [xbd[1], ybd[1], zbd[1]],
                   [xbd[2], ybd[2], zbd[2]]])


    R1 = (np.dot(np.transpose(Rd1), R) - np.dot(R, Rd1))
    Rd = Rd1
    eR = 0.5*vee(R1)

    current_omega = np.array([current_state[12],
                              current_state[13],
                              current_state[14]])

    Fd_norm_dot = np.dot(Fdesired, Fd_dot)/np.linalg.norm(Fdesired)

    zbd_dot = (np.dot(Fd_dot, np.linalg.norm(Fdesired)) - np.dot(Fdesired, Fd_norm_dot))/np.linalg.norm(Fdesired)**2

    xcd_dot = np.array([-np.sin(yawd), np.cos(yawd), 0]) * desired_state[16]

    zbd_x_xcd_dot = np.dot(skew_matrix(zbd_dot), xcd) + np.dot(skew_matrix(zbd), xcd_dot)
    zbd_xcd_norm_dot = np.dot(np.dot(skew_matrix(zbd), xcd), zbd_x_xcd_dot) / np.linalg.norm(np.dot(skew_matrix(zbd), xcd))

    ybd_dot = (zbd_x_xcd_dot*np.linalg.norm(np.dot(skew_matrix(zbd), xcd)) -
               np.dot(np.dot(skew_matrix(zbd), xcd), zbd_xcd_norm_dot)) / np.linalg.norm(np.dot(skew_matrix(zbd), xcd))**2

    xbd_dot = np.dot(skew_matrix(ybd_dot), zbd) + np.dot(skew_matrix(ybd), zbd_dot)

    Rd_dot = np.array([[xbd_dot[0], ybd_dot[0], zbd_dot[0]],
                       [xbd_dot[1], ybd_dot[1], zbd_dot[1]],
                       [xbd_dot[2], ybd_dot[2], zbd_dot[2]]])

    wd_hat = np.dot(np.transpose(Rd), Rd_dot)
    wd = vee(wd_hat)

    ew = current_omega - np.dot(np.dot(np.transpose(R), Rd), wd)


    # Moment
    # ref: Minimum Snap Trajectory Generation and Control for Quadrotors by Daniel Mellinger and Vijay Kumar
    M = - KR*eR - K_omega*ew

    u2 = M[0]
    u3 = M[1]
    u4 = M[2]

    # print u1, u2, u3, u4
    #return u1, u2, u3, u4, Fd_dot
    return u1, u2, u3, u4, Fdesired/m

if __name__ == '__main__':

    class param:
        def __init__(self):
            self.mass = 1.477
            self.grav = 9.807


    class K_param:
        def __init__(self):
            self.Kp = 5
            self.Kv = 5
            self.Kr = 5
            self.K_omega = 5

    class Q:
        def __init__(self):
            self.w = 1
            self.x = 0
            self.y = 0
            self.z = 0

    model_param = param()
    K = K_param()
    current_q = Q()

    current_state = np.array([10, 20, 30, 40, 50, 6, 7, 8, 9, 10, 11, 12, 103, 14, 15, 16, 17])
    desired_state = np.array([10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170])
    controller(current_state, desired_state, model_param, K, current_q)

    # ref: Minimum Snap Trajectory Generation and Control for Quadrotors by Daniel Mellinger and Vijay Kumar

    # omega is used for current drone state, while yaw and yaw_dot is used for desired state
    #         0  1  2  3   4   5   6    7    8    9     10    11    12       13       14       15   16
    # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', omega.x, omega.y, omega.z, yaw, yaw_dot]
    # only position, velocity and angular speed is required from drone current state
