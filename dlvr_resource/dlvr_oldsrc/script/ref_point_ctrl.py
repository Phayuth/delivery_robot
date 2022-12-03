#! /usr/bin/env python

import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

def callback(msg):
	# Reference point
	xref = 3
	yref = 3
	tref = np.arctan2(yref,xref)

	# Current pose of robot
	xcur = msg.pose.pose.position.x
	ycur = msg.pose.pose.position.y
	(r,p,tcur) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

	# Find error between ref point and current point
	delx = xref - xcur
	dely = yref - ycur
	delt = tref - tcur

	#control
	k1 = 4
	k2 = 0.1
	wc = k1*np.arctan(np.tan(delt))
	vc = k2*(np.sqrt((delx**2) + (dely**2)))*np.sign(np.cos(delt))
	rospy.loginfo("Vc = " + str(wc) + ", Wc = "+ str(wc))

	# Forward kinematic
	L = 0.44
	r = 0.07
	wl = (2*vc - L*wc)/(2*r)
	wr = (2*vc + L*wc)/(2*r)

	# publish to command robot
	left_pub = rospy.Publisher("/", Float32, queue_size = 10)
	right_pub = rospy.Publisher("/", Float32, queue_size = 10)

	left.data = wl
	right.data = wr

	left_pub.publish(left)
	right_pub.publish(right)

def main():
	rospy.init_node("Controller_node")
	left = Float32()
	right = Float32()
	rospy.Subscriber("/Odom_encd",Odometry,callback)
	rospy.spin()

if __name__ == '__main__':
	main()