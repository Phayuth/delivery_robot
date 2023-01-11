#! /usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from trajectory_msgs.msg import JointTrajectory
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def angnorm(theta):
	if theta>0:
		if theta>np.pi:
			theta = (2*np.pi-theta)*(-1)

	if theta<0:
		if theta*(-1)>np.pi:
			theta = (2*np.pi-(-theta))*(-1)

	return theta

class backstepping_controller:

	def __init__(self,k1,k2,k3,ka,kb,m,r,b,Iner):
		self.k1   = k1
		self.k2   = k2
		self.k3   = k3
		self.ka   = ka
		self.kb   = kb
		self.m    = m
		self.r    = r
		self.b    = b
		self.Iner = Iner

	def error(self,qr,qc):
		theta = qc[2,0]

		T = np.array([[np.cos(theta),np.sin(theta),0],
		             [-np.sin(theta),np.cos(theta),0],
		             [      0       ,     0       ,1]])

		e = qr - qc
		return T.dot(e)

	def controlkinematic(self,qe,vr,wr):
		return ((vr*np.cos(qe[2,0]))+self.k1*qe[0,0]) , (wr+(self.k2*vr*qe[1,0])*(self.k3*np.sin(qe[2,0])))

def odom_callback(msg):

	# Find Current Pose
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quaternion = msg.pose.pose.orientation
	ol = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
	(roll,pitch,yaw)=euler_from_quaternion(ol)
	qc = np.array([[x],[y],[yaw]])

	# Find Desired Pose
    # TO DO get xRef,yRef,theta_ref,vr,wr,ydot,xdot,vdotref,wdotref from trajectory_msgs/JointTrajectory
	qr = np.array([[xRef],[yRef],[theta_ref]])

	# Control init for each trajectory
	controller = backstepping_controller(0.7,20,20,100,3000,4,0.1,0.26,2.5)

	# Find error and control
	qe = controller.error(qr,qc)
	vc,wc = controller.controlkinematic(qe,vr,wr)

	# Print some output
	rospy.loginfo("thetaref = "+str(theta_ref)+"       thetac = "+str(qc[2,0]))

	# Publish to ROS
	cmd_pub = rospy.Publisher("/dlvr/cmd_vel", Twist, queue_size = 1)
	cmd_val = Twist()
	cmd_val.linear.x = vc
	cmd_val.angular.z = wc
	cmd_pub.publish(cmd_val)

def main():

	rospy.init_node('base_controller')
	rospy.Subscriber('/dlvr/odom',Odometry, odom_callback)
	rospy.spin()

if __name__=='__main__':
	main()