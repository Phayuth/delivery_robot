#! /usr/bin/env python
import time
import math

import rospy
from std_msgs.msg import Float32, Int32

class view_omega(object):
	def __init__(self):
		self.thetaright_old = 0
		self.thetaright_now = 0
		self.wr = 0

		self.thetaleft_old = 0
		self.thetaleft_now = 0
		self.wl = 0

		self.tsr_old = rospy.Time.now().to_sec()
		self.tsr_now = rospy.Time.now().to_sec()
		self.tsl_old = rospy.Time.now().to_sec()
		self.tsl_now = rospy.Time.now().to_sec()

		self.looprate = rospy.Rate(100)

		rospy.Subscriber('/right_ticks',Int32,self.callb_right)
		rospy.Subscriber('/left_ticks',Int32,self.callb_left)

	def callb_right(self,msg):
		self.tsr_now = rospy.Time.now().to_sec()
		Ts = self.tsr_now - self.tsr_old
		GR = 72
		PPR = 28
		self.thetaright_now = (2*math.pi*msg.data)/(GR*PPR)
		self.wr = self.thetaright_now - self.thetaright_old
		self.tsr_old = self.tsr_now
		self.thetaright_old = self.thetaright_now

	def callb_left(self,msg):
		self.tsl_now = rospy.Time.now().to_sec()
		Ts = self.tsl_now - self.tsl_old
		GR = 72
		PPR = 28
		self.thetaleft_now = (2*math.pi*msg.data)/(GR*PPR)
		self.wl = self.thetaleft_now - self.thetaleft_old
		self.tsl_old = self.tsl_now
		self.thetaleft_old = self.thetaleft_now

	def start(self):
		while not rospy.is_shutdown():

			rospy.loginfo("wr = " + str(self.wr)+"             wl = " + str(self.wl))

			self.looprate.sleep()

if __name__ == '__main__':
	rospy.init_node("view_w")
	oo = view_omega()
	oo.start()