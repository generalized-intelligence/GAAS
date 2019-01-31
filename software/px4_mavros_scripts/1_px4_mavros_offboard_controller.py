from Px4Mavros import px4controller, commander
import threading
import time



if __name__ == '__main__':

    px4 = px4controller.Px4Controller()
    px4.start()
