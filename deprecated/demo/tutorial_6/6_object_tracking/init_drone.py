from commander import Commander
import time
import rospy

con = Commander()

time.sleep(0.5)

con.move(-5,0,0)

time.sleep(0.5)
