#! /usr/bin/env python
import numpy as np
import time

import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion

class forward_kinematic(object):
	def __init__(self):
		self.thetaright_old = 0
		self.thetaleft_old = 0
		self.thetaright_now = 0
		self.thetaleft_now = 0

		self.time_old = rospy.Time.now()
		self.time_now = rospy.Time.now()

		self.x = 0
		self.y = 0
		self.t = 0

		self.looprate = rospy.Rate(100)

		rospy.Subscriber('/right_theta',Float32,self.callb_right)
		rospy.Subscriber('/left_theta',Float32,self.callb_left)

		self.odom_pub = rospy.Publisher("/odom_encder",Odometry,queue_size=10)

	def callb_right(self,msg):
		self.thetaright_now = msg.data

	def callb_left(self,msg):
		self.thetaleft_now = msg.data

	def start(self):
		while not rospy.is_shutdown():
			self.time_now = rospy.Time.now()
			Ts = self.time_now.to_sec() - self.time_old.to_sec()

			wr = (self.thetaright_now - self.thetaright_old)/Ts
			wl = (self.thetaleft_now - self.thetaleft_old)/Ts

			L = 0.44
			r = 0.07
			v = (r/2)*(wr+wl)
			w = (r/L)*(wr-wl)

			self.x = self.x + v*np.cos(self.t)*Ts
			self.y = self.y + v*np.sin(self.t)*Ts
			self.t = self.t + w*Ts
			
			odom = Odometry()
			odom.header.stamp = self.time_now
			odom.header.frame_id = "Odom_frame"
			odom.child_frame_id = "Base_link"
			quat = tf.transformations.quaternion_from_euler(0,0,self.t)
			odom.pose.pose = Pose(Point(self.x,self.y,0.),Quaternion(*quat))

			tf_broadcaster = tf.TransformBroadcaster()
			tf_broadcaster.sendTransform((self.x,self.y,0.),quat,rospy.Time.now(),"Base_link","Odom_frame")
			
			self.odom_pub.publish(odom)

			self.thetaright_old = self.thetaright_now
			self.thetaleft_old = self.thetaleft_now
			self.time_old = self.time_now

			self.looprate.sleep()

if __name__ == '__main__':
	rospy.init_node("odom_encd")
	oo = forward_kinematic()
	oo.start()