#!/usr/bin/env python
from __future__ import print_function
import xbox
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# create ros node and publisher
rospy.init_node("cmd_v_xbox")
left_pub = rospy.Publisher("/left_cmd_vel",Float32,queue_size=10)
right_pub = rospy.Publisher("/right_cmd_vel",Float32,queue_size=10)
left = Float32()
right = Float32()
joy = xbox.Joystick()

while not rospy.is_shutdown():

	solo_left = joy.leftY()
	solo_right = joy.rightY()
		
	# Joystick deadzone
	if -0.015 < solo_left < 0.015:
		solo_left = 0
	if -0.015 < solo_right < 0.015:
		solo_right = 0
	
	left.data = solo_left*250
	right.data = solo_right*250
	
	left_pub.publish(left)
	right_pub.publish(right)
	
	rospy.sleep(0.05) # in second