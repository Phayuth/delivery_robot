// Include library
#define USE_USBCON
#include <ros.h>
#include <dlvr_robot_msg/motor_stat.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>

// Time interval for handling control loop
unsigned long time_old, time_new;
float Ts;

// Time interval for handling publish
const int motor_interval = 30;
unsigned long motor_previousMillis = 0;
unsigned long motor_currentMillis = 0;

float pi = 3.141592653;

// Left Motor and Encoder
#define L_pwm_pin 5
#define L_dir_pin 4
int L_counter = 0;
uint8_t L_enc_A = 10;
uint8_t L_enc_B = 11;

// Left Motor Parameters
float L_a = 26.63;
float L_b = 17.26;
float L_PPR = 28;
float L_GR = 19.2 * 3.75;
float L_theta_p = 0;
float L_theta_c = 0;
float L_omega_d = 0;
float L_omega_d_p = 0;
float L_omega_c = 0;

// Left Motor PID Parameters
float L_kp = (2 * 1 * 2 * pi * 6 - L_a) / L_b;
float L_ki = (2 * pi * 6 * 2 * pi * 6) / L_b;
float L_cumError = 0;
float L_error = 0;
float L_u = 0;
int L_pwm;

// -----------------------------------------------------------------------------

// Right Motor and Encoder
#define R_pwm_pin 6
#define R_dir_pin 7
int R_counter = 0;
uint8_t R_enc_A = 8;
uint8_t R_enc_B = 9;

// Right Motor Parameters
float R_a = 28.37;
float R_b = 19.15;
float R_PPR = 28;
float R_GR = 19.2 * 3.75;
float R_theta_p = 0;
float R_theta_c = 0;
float R_omega_d = 0;
float R_omega_d_p = 0;
float R_omega_c = 0;

// Right Motor PID Parameters
float R_kp = (2 * 1 * 2 * pi * 6 - R_a) / R_b;
float R_ki = (2 * pi * 6 * 2 * pi * 6) / R_b;
float R_cumError = 0;
float R_error = 0;
float R_u = 0;
int R_pwm;

// ROS -------------------------------------------------------------------------
ros::NodeHandle nh;
void left_cmd_pwm_cb(const std_msgs::Float32& msg); // function prototype
void right_cmd_pwm_cb(const std_msgs::Float32& msg); // function prototype

// Publish Motor Stat
dlvr_robot_msg::motor_stat pub_motor_stat;
ros::Publisher motorPub("/dlvr/motor_stat", &pub_motor_stat);

// Subscribe Left/Right wheel velocity
ros::Subscriber<std_msgs::Float32> left_sub("/dlvr/left_motor_desired", &left_cmd_pwm_cb);
ros::Subscriber<std_msgs::Float32> right_sub("/dlvr/right_motor_desired", &right_cmd_pwm_cb);
// ROS -------------------------------------------------------------------------

void setup() {
  // Motor
  pinMode(L_pwm_pin, OUTPUT);
  pinMode(L_dir_pin, OUTPUT);
  pinMode(R_pwm_pin, OUTPUT);
  pinMode(R_dir_pin, OUTPUT);

  // Interupt encoder
  pinMode(L_enc_A, INPUT_PULLUP);
  pinMode(L_enc_B, INPUT_PULLUP);
  pinMode(R_enc_A, INPUT_PULLUP);
  pinMode(R_enc_B, INPUT_PULLUP);
  attachInterrupt(L_enc_A, ai10, CHANGE);
  attachInterrupt(L_enc_B, ai11, CHANGE);
  attachInterrupt(R_enc_A, ai8, CHANGE);
  attachInterrupt(R_enc_B, ai9, CHANGE);

  // Time Record
  time_old = millis();
  time_new = millis();

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(left_sub);
  nh.subscribe(right_sub);
  nh.advertise(motorPub);
}

void loop() {
  // Find out how time has passed
  delay(1);
  time_new = millis(); // handle control loop
  motor_currentMillis = millis(); // handle publisher loop

  Ts = (time_new - time_old) * 0.001; // convert from milli second (ms) to second(s)

  // Find current theta and find omega
  R_theta_c = tick_2_theta(R_counter, R_PPR, R_GR);
  R_omega_c = (R_theta_c - R_theta_p) / Ts;
  L_theta_c = tick_2_theta(L_counter, L_PPR, L_GR);
  L_omega_c = (L_theta_c - L_theta_p) / Ts;

  // Calculate different error
  R_error = R_omega_d - R_omega_c;
  L_error = L_omega_d - L_omega_c;

  // Calculate input voltage
  R_u = (R_kp * R_error) + (R_ki * (R_cumError + R_error * Ts)) + ((R_a / R_b) * R_omega_d) + ((1 / R_b) * ((R_omega_d - R_omega_d_p) / Ts));
  L_u = (L_kp * L_error) + (L_ki * (L_cumError + L_error * Ts)) + ((L_a / L_b) * L_omega_d) + ((1 / L_b) * ((L_omega_d - L_omega_d_p) / Ts));

  // Write control value to motor
  R_pwm = pwm_cal(R_u, 23.5);
  L_pwm = pwm_cal(L_u, 23.5);
  R_write_pwm(R_pwm);
  L_write_pwm(L_pwm);

  //Publish stat
  if (motor_currentMillis - motor_previousMillis > motor_interval)
  {
    motor_previousMillis = motor_currentMillis;

    pub_motor_stat.header.stamp = nh.now();
    pub_motor_stat.header.frame_id = "motor";
    pub_motor_stat.speed_left = L_omega_c;
    pub_motor_stat.encoder_left = L_counter;
    pub_motor_stat.speed_right = R_omega_c;
    pub_motor_stat.encoder_right = R_counter;

    motorPub.publish( &pub_motor_stat );
  }

  // Update for next time step
  time_old = time_new;

  L_theta_p = L_theta_c;
  R_theta_p = R_theta_c;

  L_cumError += L_error * Ts;
  R_cumError += R_error * Ts;

  L_omega_d_p = L_omega_d;
  R_omega_d_p = R_omega_d;

  nh.spinOnce();
}
