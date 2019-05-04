import numpy as np

from bresenham3d import Bresenham3D

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class PathPruning:

    def __init__(self, obstacle_distance=10):

        self.obstacle_distance = obstacle_distance

    '''
    for a path where there are multiple points that are collinear
    keep the first and last points and remove the rest.
    '''
    def remove_collinear_points(self, original_path):

        original_path = self.Remove(original_path)
        new_path = []
        length = len(original_path)
        new_path.append(original_path[0])

        for i in range(length)[2:]:

            distance13 = self.distance(original_path[i], original_path[i-2])
            distance12 = self.distance(original_path[i-1], original_path[i-2])
            distance23 = self.distance(original_path[i], original_path[i-1])

            if abs(distance13 - distance12 - distance23) < 0.001:
                continue

            else:
                new_path.append(original_path[i-1])

        new_path.append(original_path[-1])

        return new_path

    '''
    1. given a path: [a, b, c, d];
    2. iterate points in a way like [a, b], [a, c], [a, d], [b, c] ...
    3. draw a line with bresenham and see if obstacle point is in it
    4. if not, remove middle point and continue
    '''
    def path_pruning_bresenham3d(self, path, local_obstacle):

        final_path = []
        start_position = path[0]
        final_path.append(start_position)
        last_point = start_position

        for point in path:

            bresenham_path = Bresenham3D(start_position, point)

            for ob in local_obstacle:
                for path_point in bresenham_path:
                    if self.distance(ob, path_point) < self.obstacle_distance:

                        #print("ob in local path: ", ob)
                        final_path.append(last_point)
                        final_path.append(point)
                        start_position = point

            last_point = point

        final_path.append(path[-1])
        final_path = self.Remove(final_path)

        return final_path


    '''
    path smoothing using B spline
    '''
    def BSplinePathSmoothing(self, path):
        pass


    '''
    helper functions
    '''
    def Remove(self, duplicate):
        final_list = []
        for num in duplicate:
            if num not in final_list:
                final_list.append(num)
        return final_list


    def distance(self, p1, p2):
        x_distance = (p2[0] - p1[0])**2
        y_distance = (p2[1] - p1[1])**2
        z_distance = (p2[2] - p1[2])**2

        return np.sqrt(x_distance + y_distance + z_distance)



if __name__ == '__main__':

    def draw_path(path, ax, color='r', z_offset=0):
        x = []
        y = []
        z = []
        for point in path:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2] + z_offset)

        ax.scatter(x, y, z, c=color, s=100)
        ax.plot(x, y, z, color=color)

    # generate a flight path
    path = []
    for i in range(10):
        path.append((i, 0, 0))

    for i in range(10):
        path.append((9, i, 0))

    for i in range(5):
        path.append((9, 9, i))

    # generate a obstacle
    obstacle = []
    for i in range(1, 5)[2:]:
        for j in range(1, 5)[2:]:
            for k in range(1, 16)[6:]:
                obstacle.append((i, j, k))


    ax = plt.gca(projection="3d")
    PathPruning = PathPruning(7.1)

    draw_path(obstacle, ax, 'r')
    draw_path(path, ax, 'r')

    # 1. remove collinear points
    processed_path_1 = PathPruning.remove_collinear_points(path)

    print("processed_path_1: ", processed_path_1)

    draw_path(processed_path_1, ax, 'g', 5)

    # 2. use raytracing to remove extra points
    processed_path_2 = PathPruning.path_pruning_bresenham3d(path, obstacle)

    print("processed_path_2: ", processed_path_2)

    draw_path(processed_path_2, ax, 'b', 10)

    plt.show()
