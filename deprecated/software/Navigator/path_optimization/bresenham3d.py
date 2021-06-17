import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# this function is derived from the following link:
# https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing/


def Bresenham3D(p1, p2):

    (x1, y1, z1) = p1
    (x2, y2, z2) = p2

    ListOfPoints = []
    ListOfPoints.append((x1, y1, z1))

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)

    if (x2 > x1):
        xs = 1
    else:
        xs = -1

    if (y2 > y1):
        ys = 1
    else:
        ys = -1

    if (z2 > z1):
        zs = 1
    else:
        zs = -1

    # Driving axis is X-axis"
    if (dx >= dy and dx >= dz):
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while (x1 != x2):
            x1 += xs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dx
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            ListOfPoints.append((x1, y1, z1))

    # Driving axis is Y-axis"
    elif (dy >= dx and dy >= dz):
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while (y1 != y2):
            y1 += ys
            if (p1 >= 0):
                x1 += xs
                p1 -= 2 * dy
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            ListOfPoints.append((x1, y1, z1))

    # Driving axis is Z-axis"
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while (z1 != z2):
            z1 += zs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dz
            if (p2 >= 0):
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            ListOfPoints.append((x1, y1, z1))
    return ListOfPoints


if __name__ == '__main__':
    p1 = (-1, 1, 1)
    p2 = (5, 3, -1)
    ListOfPoints = Bresenham3D(p1, p2)
    print(ListOfPoints)

    print("start position: ", p1)
    print("end posiiton: ", p2)

    x = []
    y = []
    z = []
    for point in ListOfPoints:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    ax = plt.gca(projection="3d")
    ax.scatter(x, y, z, c='r', s=100)
    ax.plot(x,y,z, color='r')

    plt.show()


