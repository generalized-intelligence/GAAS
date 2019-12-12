import numpy as np
import matplotlib.pyplot as plt


def GenerateCirclePathPoints(radius=10):
    points = []
    x = 0
    while(x <= radius):
        y = np.sqrt(radius*radius - x*x)
        points.append([x, y])
        points.append([x, -y])
        points.append([-x, y])
        points.append([-x, -y])
        x += 0.01
    return points

def LateralAccCommand(V, Theta, L1=2):
    a = 2*V*V*np.sin(Theta)/L1
    return a

def FindTheta(drone_position, v_vec, trajectory_pts, L1=2):
    assert(len(drone_position) == 2)

    #print drone_position
    #print v_vec

    reference_pt = None
    theta = None

    temp = 1000
    for pt in trajectory_pts:
        delta_x = (drone_position[0] - pt[0]) ** 2
        delta_y = (drone_position[1] - pt[1]) ** 2
        distance = np.sqrt(delta_x + delta_y)

        if abs(distance - L1) < temp:
            temp = abs(distance - L1)

        if(abs(distance - L1)<= 0.1):
            reference_pt = pt
            reference_vec = reference_pt - drone_position

            length_refer = np.sqrt(reference_vec[0]**2+reference_vec[1]**2)
            length_v = np.sqrt(v_vec[0]**2+v_vec[1]**2)

            theta = np.arccos((reference_vec[0]*v_vec[0]+reference_vec[1]*v_vec[1])/(length_refer + length_v))

            if(theta > np.pi/2):
                continue

            dir = np.cross(np.array(reference_vec), np.array(v_vec))

            print "theta: ", theta*180/np.pi
            if(dir<0):
                return theta
            else:
                return -theta

    print ("temp: ", temp)
    print ("theta is None, quit!")

def LateralSpeedVec(forward, L):

def Test():

    forward_speed = np.array([1, 1])
    cur_position = np.array([0, -12])
    acc_lateral = np.array([0, 0])
    total_speed = forward_speed

    traj_pts = GenerateCirclePathPoints()

    actual_path = []
    i = 0
    while(i < 30):
        print "cur_position: ", cur_position

        theta = FindTheta(cur_position, forward_speed, traj_pts, 2)
        acc_lateral = LateralAccCommand(np.sqrt(forward_speed[0]**2 + forward_speed[1]**2), theta, 2)

        cur_position = cur_position + total_speed * 0.1 + 0.5 * acc_lateral*0.1**2

        actual_path.append([cur_position[0], cur_position[1]])

        total_speed = total_speed + acc_lateral * 0.1
        print "total_speed: ", total_speed

        i += 1
    return actual_path




if __name__ == '__main__':

    pts = GenerateCirclePathPoints()
    xs = []
    ys = []
    for pt in pts:
        xs.append(pt[0])
        ys.append(pt[1])

    actual_pts = Test()
    xa = []
    ya = []
    for pt in actual_pts:
        xa.append(pt[0])
        ya.append(pt[1])

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(xs, ys, c='r', marker='o')
    ax.scatter(xa, ya, c='g', marker='o')
    plt.show()



