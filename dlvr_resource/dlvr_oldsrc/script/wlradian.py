#! /usr/bin/env python
import math
import rospy
from std_msgs.msg import Int32, Float32

def pulse2radian(count_in):
	GR = 72
	PPR = 28
	theta = (2*math.pi*count_in)/(GR*PPR)
	return theta

def callb_left(msg):
	theta_left = pulse2radian(msg.data)
	pub_l = rospy.Publisher("/left_theta",Float32,queue_size=10)
	left = Float32()
	left.data = theta_left
	pub_l.publish(left)

def callb_right(msg):
	theta_right = pulse2radian(msg.data)
	pub_r = rospy.Publisher("/right_theta",Float32,queue_size=10)
	right = Float32()
	right.data = theta_right
	pub_r.publish(right)

def main():
	rospy.init_node('p2r')
	rospy.Subscriber("/left_ticks",Int32,callb_left)
	rospy.Subscriber("/right_ticks",Int32,callb_right)
	rospy.spin()

if __name__ == '__main__':
	main()