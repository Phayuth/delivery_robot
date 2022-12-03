#include <ros/ros.h>
#include <dlvr_robot_hardware/pub2sub1.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


double L = 0.44;
double r = 0.07;

double V;
double omega;
double L_omega_d;
double R_omega_d;

template<>
void Publisher2Subscriber1<std_msgs::Float32, std_msgs::Float32, geometry_msgs::Twist>::subscriberCallback(const geometry_msgs::Twist::ConstPtr& recievedMsg)
{
	V = recievedMsg->linear.x;
	omega = recievedMsg->angular.z;

	L_omega_d = (2*V-L*omega)/(2*r);
	R_omega_d = (2*V+L*omega)/(2*r);

	std_msgs::Float32 L_omega_desired;
	std_msgs::Float32 R_omega_desired;

	L_omega_desired.data = L_omega_d;
	R_omega_desired.data = R_omega_d;

	publisher1Object.publish(L_omega_desired);
	publisher2Object.publish(R_omega_desired);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Twist_to_wheel");
	
	ROS_INFO("Starting");
	Publisher2Subscriber1<std_msgs::Float32, std_msgs::Float32, geometry_msgs::Twist> twist_to_wheel("/dlvr/left_motor_desired","/dlvr/right_motor_desired","cmd_vel",1);

	ros::spin();
}