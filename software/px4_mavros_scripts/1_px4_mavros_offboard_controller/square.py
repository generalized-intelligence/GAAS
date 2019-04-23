from commander import Commander
import time


con = Commander()
time.sleep(2)


# use FLU body frame

print "moving 3 meter to the right"
con.move(0, -3, 0)
time.sleep(8)

print "turn 90 degrees anticlockwise"
con.turn(90)
time.sleep(10)


print "moving 3 meter to the right"
con.move(0, -3, 0)
time.sleep(8)

print "turn 90 degrees anticlockwise"
con.turn(180)
time.sleep(10)


print "moving 3 meter to the right"
con.move(0, -3, 0)
time.sleep(8)

print "turn 90 degrees anticlockwise"
con.turn(270)
time.sleep(10)

print "moving 3 meter to the right"
con.move(0, -3, 0)
time.sleep(8)

print "turn 90 degrees anticlockwise"
con.turn(360)
time.sleep(10)

time.sleep(3)

# land!
print "Land!"
con.land()



