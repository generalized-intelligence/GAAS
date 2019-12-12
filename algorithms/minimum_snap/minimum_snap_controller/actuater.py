import rospy
from mavros_msgs.msg import ActuatorControl

if __name__ == '__main__':
	# Setup the ROS backend for this node
	rospy.init_node('actuator_controller', anonymous=True)
	# Setup the publisher that will send data to mavros
	pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

	# Configure our program (will turn actuators on/off at 1Hz)
	rate = rospy.Rate(0.5)
	is_low = True


	all_actuators_on = [0,0,0,0.55,0,0,0,0]

	try:
		while not rospy.is_shutdown():
			# Prepare message to send to MAVROS
			# http://docs.ros.org/api/mavros_msgs/html/msg/ActuatorControl.html
			msg_out = ActuatorControl()
			msg_out.header.stamp = rospy.Time.now()
			msg_out.group_mix = 0

			if 0:
				msg_out.controls = all_actuators_off
				rospy.loginfo("Set servos low")
			else:
				msg_out.controls = all_actuators_on
				rospy.loginfo("Set servos high")

			# Publish the complete message to MAVROS
			pub.publish(msg_out)

			# Sleep for a bit and repeat while loop
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass
