#include <ros/ros.h>
#include <dlvr_robot_msg/motor_stat.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// Motor Parameters
double L_GR = 72;
double L_PPR = 23;
double R_GR = 72;
double R_PPR = 23;

double L_theta_old = 0;
double R_theta_old = 0;
double L_theta_now = 0;
double R_theta_now = 0;

int enc_left;
int enc_right;

// Odometry Parameters
double pi = 3.14;
double v;
double omega;
double L = 0.44;
double r = 0.07;

double x = 0.0;
double y = 0.0;
double th = 0.0;

void chatterCallback(const dlvr_robot_msg::motor_stat::ConstPtr& msg)
{
  enc_left = msg->encoder_left;
  enc_right = msg->encoder_right;

  L_theta_now = (2*pi*enc_left)/(L_GR*L_PPR);
  R_theta_now = (2*pi*enc_right)/(R_GR*R_PPR);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odometry_publisher");
  ROS_INFO("STARTING Odometry Publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber sub = n.subscribe("/dlvr/motor_stat", 1, chatterCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rosrate(100.0);

  // ros::spin();

  while(n.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    double L_omega = ( L_theta_now - L_theta_old) / dt;
    double R_omega = ( R_theta_now - R_theta_old) / dt;

    v = (R_omega + L_omega)*r/2;
    omega = (R_omega - L_omega)*r/L;

    double delta_x = (v * cos(th)) * dt;
    double delta_y = (v * sin(th)) * dt;
    double delta_th = omega * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    L_theta_old = L_theta_now;
    R_theta_old = R_theta_now;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = omega;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    rosrate.sleep();
  }
}