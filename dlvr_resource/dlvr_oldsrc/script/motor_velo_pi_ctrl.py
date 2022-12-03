#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32

class motor_control(object):
	def __init__(self):
		# Right MOTOR ===============================================================
		# Motor Parameter
		self.a = 27.65
		self.b = 17.35
		self.c = 5.635

		# PID
		self.pi = 3.141592653
		self.wn = 2*self.pi*6
		self.zeta = 1
		# self.kp = (2*self.zeta*self.wn - self.a)/self.b
		# self.ki = (self.wn**2)/self.b
		self.kp = 2
		self.ki = 0.5

		# variable
		self.tsr_old = rospy.Time.now().to_sec()
		self.tsr_now = rospy.Time.now().to_sec()
		
		self.thetaright_old = 0
		self.thetaright_now = 0
		# self.wr = 0
		
		self.cumError = 0
		self.wrdp = 0
		# self.ur = 0
		# Right MOTOR ===============================================================

		self.looprate = rospy.Rate(100)

		rospy.Subscriber('/right_ticks',Int32,self.callb_right)

		self.right_pwm_pub = rospy.Publisher("/right_cmd_pwm",Int32,queue_size=1)

	def callb_right(self,msg):
		self.tsr_now = rospy.Time.now().to_sec()
		Ts = self.tsr_now - self.tsr_old
		GR = 72
		PPR = 28
		self.thetaright_now = (2*self.pi*msg.data)/(GR*PPR)
		wr = (self.thetaright_now - self.thetaright_old)/Ts

		# control
		wrd = 5
		error = wrd - wr
		rospy.loginfo("wr = "+ str(wr))
		ur = (self.kp * error) + (self.ki * (self.cumError + error * Ts)) + ((self.a / self.b) * wrd) + ((1 / self.b) * ((wrd - self.wrdp) / Ts))

		rpwm = Int32()
		rpwm.data = self.cal_pwm(ur,23)
		self.right_pwm_pub.publish(rpwm)

		self.tsr_old = self.tsr_now
		self.thetaright_old = self.thetaright_now
		self.cumError += error * Ts
		self.wrdp = wrd

	def cal_pwm(self,u,v_max):
		pwm = (u/v_max)*254
		if pwm<-254:
			pwm = -254
		elif pwm>254:
			pwm = 254
		return pwm

  	def start(self):
  		while not rospy.is_shutdown():
			
			self.looprate.sleep()

if __name__ == '__main__':
	rospy.init_node("motor_velo_controller")
	mc = motor_control()
	mc.start()