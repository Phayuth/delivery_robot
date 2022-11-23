#!/usr/bin/env python
from __future__ import print_function
import xbox
import rospy
from geometry_msgs.msg import Twist

# create ros node and publisher
rospy.init_node("cmd_v_xbox")
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
twt = Twist()
joy = xbox.Joystick()

while not rospy.is_shutdown():
	a = joy.leftY()
	b = joy.rightX()
	
	# Joystick deadzone
	if -0.015 < a < 0.015:
		a = 0
	if -0.015 < a < 0.015:
		b = 0
	
	twt.linear.x = a*1
	twt.angular.z = b*1
	
	pub.publish(twt)
	rospy.sleep(0.03) # in second