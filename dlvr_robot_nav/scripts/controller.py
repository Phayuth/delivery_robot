#! /usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from trajectory_msgs.msg import JointTrajectory
import tf.transformations as tf


def angnorm(theta):
	if theta>0:
		if theta>np.pi:
			theta = (2*np.pi-theta)*(-1)

	if theta<0:
		if theta*(-1)>np.pi:
			theta = (2*np.pi-(-theta))*(-1)

	return theta

class controller:

	def __init__(self):

		self.k1   = 0.7
		self.k2   = 20
		self.k3   = 20
		self.ka   = 100
		self.kb   = 3000
		self.m    = 4
		self.r    = 0.1
		self.b    = 0.26
		self.Iner = 2.5
		
		self.qc = np.array([[0],[0],[0]])
		self.qr = np.array([[0],[0],[0]])

		self.vr = 0
		self.wr = 0

		rospy.init_node('base_controller')
		rospy.Subscriber('/dlvr/odom',Odometry, self.odom_callback)
		rospy.Subscriber('/dlvr/desired_pose',JointTrajectory, self.desired_callback)

		self.looprate = rospy.Rate(100)
		
	def error(self,qr,qc):
		theta = qc[2,0]

		T = np.array([[np.cos(theta),np.sin(theta),0],
		             [-np.sin(theta),np.cos(theta),0],
		             [      0       ,     0       ,1]])

		e = qr - qc
		return T.dot(e)

	def controlkinematic(self,qe,vr,wr):
		return ((vr*np.cos(qe[2,0]))+self.k1*qe[0,0]) , (wr+(self.k2*vr*qe[1,0])*(self.k3*np.sin(qe[2,0])))

	def desired_callback(self,msg):
		# Get Current Pose
		xd = msg.points[0].positions[0]
		yd = msg.points[0].positions[1]

		xdot = msg.points[0].velocities[0]
		ydot = msg.points[0].velocities[1]

		xddot = msg.points[0].accelerations[0]
		yddot = msg.points[0].accelerations[1]

		theta_d = np.arctan2(ydot, xdot)

		self.qr = np.array([[xd],[yd],[theta_d]])
		self.vr = np.sqrt(((xdot**2) + (ydot**2)))
		self.wr = ((xdot*yddot)-(ydot*xddot))/((xdot**2) + (ydot**2))

	def odom_callback(self,msg):

		# Find Current Pose
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		quaternion = msg.pose.pose.orientation
		quaternion_val = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
		(roll,pitch,yaw)=tf.euler_from_quaternion(quaternion_val)
		self.qc = np.array([[x],[y],[yaw]])

	def start(self):
		while not rospy.is_shutdown():

			# calculate error
			qe = self.error(self.qr,self.qc)
			vc,wc = self.controlkinematic(qe,self.vr,self.wr)

			# Publish to ROS
			cmd_pub = rospy.Publisher("/dlvr/cmd_vel", Twist, queue_size = 1)
			cmd_val = Twist()
			cmd_val.linear.x = vc
			cmd_val.angular.z = wc
			cmd_pub.publish(cmd_val)
			
			self.looprate.sleep()

if __name__=='__main__':
	ct = controller()
	ct.start()