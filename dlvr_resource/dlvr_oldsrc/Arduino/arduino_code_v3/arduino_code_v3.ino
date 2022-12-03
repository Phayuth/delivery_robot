// Include library
#define USE_USBCON // For Arduino Due rosserial in every board and computer
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Init IMU object
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x28);

// Encoder and Motor Left
#define left_motor_pwm 5
#define left_motor_dir 4
int counter_left = 0;
uint8_t enc_left_A = 10;
uint8_t enc_left_B = 11;

// Encoder and Motor Right
#define right_motor_pwm 6
#define right_motor_dir 7
int counter_right = 0;
uint8_t enc_right_A = 8;
uint8_t enc_right_B = 9;

// preassign variable
int left_pwm;
int right_pwm;

void left_cmd_pwm_cb(const std_msgs::Int32& msg); // function prototype
void right_cmd_pwm_cb(const std_msgs::Int32& msg); // function prototype

// Init Encoder Pub
std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Init Imu Pub
sensor_msgs::Imu imu_data;
ros::Publisher imupub("imu", &imu_data);
geometry_msgs::Quaternion q;

// Init Left wheel velocity Sub
ros::Subscriber<std_msgs::Int32> left_sub("/left_cmd_pwm", &left_cmd_pwm_cb);

// Init Right wheel velocity Sub
ros::Subscriber<std_msgs::Int32> right_sub("/right_cmd_pwm", &right_cmd_pwm_cb);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void setup() {
  // Motor
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_dir, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_dir, OUTPUT);

  // Interupt encoder
  pinMode(enc_left_A, INPUT_PULLUP);
  pinMode(enc_left_B, INPUT_PULLUP);
  pinMode(enc_right_A, INPUT_PULLUP);
  pinMode(enc_right_B, INPUT_PULLUP);
  attachInterrupt(enc_left_A, ai10, CHANGE);
  attachInterrupt(enc_left_B, ai11, CHANGE);
  attachInterrupt(enc_right_A, ai8, CHANGE);
  attachInterrupt(enc_right_B, ai9, CHANGE);

  // IMU
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);

  // ROS Setup
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(left_sub); // start subscribe to sub
  nh.subscribe(right_sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(imupub);
}

void loop() {

  // Record the time
  currentMillis = millis();

  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    uint8_t system, gyro, accel, mg = 0;
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> rpy = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

    //populate data to msg
    //imu_data.header.seq = 0;
    imu_data.header.stamp = nh.now();
    imu_data.header.frame_id = "imu_link";

    q = tf::createQuaternionFromYaw(rpy.z());

    imu_data.orientation.x = q.x;
    imu_data.orientation.y = q.y;
    imu_data.orientation.z = q.z;
    imu_data.orientation.w = q.w;

    imu_data.angular_velocity.x = gyr.x();
    imu_data.angular_velocity.y = gyr.y();
    imu_data.angular_velocity.z = gyr.z();

    imu_data.linear_acceleration.x = acc.x();
    imu_data.linear_acceleration.y = acc.y();
    imu_data.linear_acceleration.z = acc.z();

    imupub.publish( &imu_data );
  }
  rightPub.publish( &right_wheel_tick_count );
  leftPub.publish( &left_wheel_tick_count );
  nh.spinOnce(); // "spinonce" have to be outside of "if" to avoid msg not publish
}
