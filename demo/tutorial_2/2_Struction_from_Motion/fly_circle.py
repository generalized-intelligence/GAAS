from commander import Commander
import time
import math
import matplotlib.pyplot as plt



class fly_circle:
    def __init__(self, Commander, height, building_radius, n):
        self.r = building_radius
        self.n = n
        self.rad_size = 2 * math.pi / self.n
        self.com = Commander
	self.height = height
    
    '''
    Projected xy value to the x and y axis
    '''
    def projected_x_y(self, idx):
        triangle_rad = idx * self.rad_size

        if triangle_rad < math.pi:

            x_rad = (math.pi / 2.0) - (math.pi - triangle_rad) / 2.0
            line_segment = math.sqrt(2*self.r**2 -2 * (self.r**2) * math.cos(triangle_rad))
            x_idx = line_segment * math.sin(x_rad)
            y_idx = -1 * line_segment * math.cos(x_rad)
            

            return x_idx, y_idx, triangle_rad

        if triangle_rad >= math.pi:

            x_rad = (math.pi / 2.0) - (triangle_rad - math.pi) / 2.0
            line_segment = math.sqrt(2*self.r**2 -2 * (self.r**2) * math.cos(triangle_rad))
            x_idx = line_segment * math.sin(x_rad)
            y_idx = line_segment * math.cos(x_rad)
            

            return x_idx, y_idx, triangle_rad

    '''
    visualize planned path
    '''
    def visualize_path(self):
        x_history = []
        y_history = []
        for idx in range(self.n + 1):
            x, y, triangle_rad = self.projected_x_y(idx)
            print(x, y, triangle_rad)
            x_history.append(x)
            y_history.append(y)

        plt.figure()
        plt.plot(x_history, y_history)
        plt.show()


    def fly(self):
        for idx in range(self.n + 1):
            x, y, triangle_rad= self.projected_x_y(idx)

            com.move( x, y, self.height, BODY_OFFSET_ENU=False)
            time.sleep(4)

            # commander.turn() requires angle in Degree
            com.turn(triangle_rad * 180.0 /math.pi)
            time.sleep(2.5)

        print("Target Reached!")


if __name__ == '__main__':

    com = Commander()

    # 1st circle
    circle = fly_circle(com, height=3, building_radius=17, n=20)
    #circle.visualize_path()
    circle.fly()
    
    # 2nd circle 
    #circle = fly_circle(com, height=5, building_radius=17, n=20)
    #circle.fly()

    # land!
    com.land()



