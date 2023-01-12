#! /usr/bin/env python

import numpy as np
import bisect
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CubicSpline1D:

	def __init__(self, x, y):

		h = np.diff(x)
		if np.any(h < 0):
			raise ValueError("x coordinates must be sorted in ascending order")

		self.a, self.b, self.c, self.d = [], [], [], []
		self.x = x
		self.y = y
		self.nx = len(x)  # dimension of x

		# calc coefficient a
		self.a = [iy for iy in y]

		# calc coefficient c
		A = self.__calc_A(h)
		B = self.__calc_B(h, self.a)
		self.c = np.linalg.solve(A, B)

		# calc spline coefficient b and d
		for i in range(self.nx - 1):
			d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
			b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
			self.d.append(d)
			self.b.append(b)

	def calc_position(self, x):

		# check if the time given is inside the segment area
		if x < self.x[0]:
			return None
		elif x > self.x[-1]:
			return None

		# check if the time given belong to what segment area
		i = self.__search_index(x)
		dx = x - self.x[i]

		# calculate the equation q(t) = a + bt + ct^2 + dt^3
		position = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

		return position

	def calc_first_derivative(self, x):
	 
		if x < self.x[0]:
			return None
		elif x > self.x[-1]:
			return None

		i = self.__search_index(x)
		dx = x - self.x[i]
		dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
		return dy

	def calc_second_derivative(self, x):

		if x < self.x[0]:
			return None
		elif x > self.x[-1]:
			return None

		i = self.__search_index(x)
		dx = x - self.x[i]
		ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
		return ddy

	def __search_index(self, x):
		"""
		search data segment index
		"""
		return bisect.bisect(self.x, x) - 1

	def __calc_A(self, h):
		"""
		calc matrix A for spline coefficient c
		"""
		A = np.zeros((self.nx, self.nx))
		A[0, 0] = 1.0
		for i in range(self.nx - 1):
			if i != (self.nx - 2):
				A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
			A[i + 1, i] = h[i]
			A[i, i + 1] = h[i]

		A[0, 1] = 0.0
		A[self.nx - 1, self.nx - 2] = 0.0
		A[self.nx - 1, self.nx - 1] = 1.0
		return A

	def __calc_B(self, h, a):
		"""
		calc matrix B for spline coefficient c
		"""
		B = np.zeros(self.nx)
		for i in range(self.nx - 2):
			B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i]
		return B

def main():
	rospy.init_node('trajectory_planner')
	pub = rospy.Publisher("/dlvr/desired_pose",JointTrajectory,queue_size = 1)
	rospy.loginfo("Starting")

	r = rospy.Rate(100)

	x = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0]
	y = [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0]
	t = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]

	spx = CubicSpline1D(t, x)
	spy = CubicSpline1D(t, y)

	t0 = rospy.Time.now().to_sec()
	tf = rospy.Time.now().to_sec()

	while not rospy.is_shutdown():
		tf = rospy.Time.now().to_sec()
		t = tf - t0

		joint = JointTrajectory()
		joint.header.stamp = rospy.Time.now()
		joint.header.frame_id = 'base'
		joint.joint_names = ['x', 'y']

		xd = spx.calc_position(t)
		yd = spy.calc_position(t)

		xdot = spx.calc_first_derivative(t)
		ydot = spy.calc_first_derivative(t)

		xddot = spx.calc_second_derivative(t)
		yddot = spy.calc_second_derivative(t)

		point = JointTrajectoryPoint()
		point.positions = [xd,yd]
		point.velocities = [xdot,ydot]
		point.accelerations = [xddot,yddot]
		point.effort = []
		point.time_from_start = rospy.Duration(1)

		joint.points.append(point)
		pub.publish(joint)

		print(t)

		r.sleep()

if __name__ == '__main__':
	main()