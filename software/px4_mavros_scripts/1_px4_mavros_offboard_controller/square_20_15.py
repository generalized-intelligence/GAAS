from commander import Commander
import time


con = Commander()
time.sleep(2)

# turn 90 anti-clockwise
print("turn to 90 deg!")
con.turn(90)
time.sleep(3)

# move left relative to BODY_OFFSET_NED frame
print("move 20m to the left!")
con.move(20,0,0)
time.sleep(20)

# turn 90 anti-clockwise
print("turn to 180 deg!")
con.turn(180)
time.sleep(3)

# move left relative to BODY_OFFSET_NED frame
print("move 15m to the left!")
con.move(15,0,0)
time.sleep(15)

# turn 90 anti-clockwise
print("turn to 270 deg!")
con.turn(270)
time.sleep(3)

# move left relative to BODY_OFFSET_NED frame
print("move 20m to the left!")
con.move(20,0,0)
time.sleep(20)

# turn 90 anti-clockwise
print("turn to 360 deg!")
con.turn(360)
time.sleep(3)

# move left relative to BODY_OFFSET_NED frame
print("move 15m to the left!")
con.move(15,0,0)
time.sleep(15)

# land!
con.land()



