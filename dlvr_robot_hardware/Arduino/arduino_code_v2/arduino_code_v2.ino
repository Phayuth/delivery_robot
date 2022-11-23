// Include library
#define USE_USBCON // For Arduino Due rosserial in every board and computer
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Init IMU object
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x28);

// Encoder and Motor Left
#define left_motor_Rpwm_pin 5
#define left_motor_Lpwm_pin 4
int counter_left = 0;
uint8_t Pin_left_A = 10;
uint8_t Pin_left_B = 11;
uint8_t Interrupt_left_A = 10;
uint8_t Interrupt_left_B = 11;

// Encoder and Motor Right
#define right_motor_Rpwm_pin 7
#define right_motor_Lpwm_pin 6
int counter_right = 0;
uint8_t Pin_right_A = 8;
uint8_t Pin_right_B = 9;
uint8_t Interrupt_right_A = 8;
uint8_t Interrupt_right_B = 9;

// preassign variable
int left_pwm; // pwm for left wheel
float left_velo; // recv velocity from controller in rad/s
int right_pwm; // pwm for right wheel
float right_velo; // recv velocity from controller in rad/s


// callback function for subcribe to left wheel
void left_cmd_vel_cb(const std_msgs::Float32& msg)
{
  // get linear and angular velocity from twist
  left_velo = msg.data;
  left_pwm = 1 * left_velo + 0;

  if (left_pwm < -250) {
    right_pwm = -250;
  }
  if (left_pwm > 250) {
    right_pwm = 250;
  }

  // write pwm to pin
  if ( left_pwm < 0) {
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, left_pwm * -1);
  }
  else if ( left_pwm > 0) {
    analogWrite(left_motor_Rpwm_pin, left_pwm);
    analogWrite(left_motor_Lpwm_pin, 0);
  }
  else {
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, 0);
  }
}


// callback function for subcribe to right wheel
void right_cmd_vel_cb(const std_msgs::Float32& msg)
{
  // get linear and angular velocity from twist
  right_velo = msg.data;
  right_pwm = 1 * right_velo + 0;

  if (right_pwm < -250) {
    right_pwm = -250;
  }
  if (right_pwm > 250) {
    right_pwm = 250;
  }

  // write pwm to pin
  if (right_pwm < 0) {
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, right_pwm * -1);
  }
  else if (right_pwm > 0) {
    analogWrite(right_motor_Rpwm_pin, right_pwm);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
  else {
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
}

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
ros::Subscriber<std_msgs::Float32> left_sub("/left_cmd_vel", &left_cmd_vel_cb);

// Init Right wheel velocity Sub
ros::Subscriber<std_msgs::Float32> right_sub("/right_cmd_vel", &right_cmd_vel_cb);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void setup() {
  // Motor
  pinMode(left_motor_Rpwm_pin, OUTPUT);
  pinMode(left_motor_Lpwm_pin, OUTPUT);
  pinMode(right_motor_Rpwm_pin, OUTPUT);
  pinMode(right_motor_Lpwm_pin, OUTPUT);

  // Interupt encoder
  pinMode(Pin_left_A, INPUT_PULLUP);
  pinMode(Pin_left_B, INPUT_PULLUP);
  pinMode(Pin_right_A, INPUT_PULLUP);
  pinMode(Pin_right_B, INPUT_PULLUP);
  attachInterrupt(Interrupt_left_A, ai10, CHANGE);
  attachInterrupt(Interrupt_left_B, ai11, CHANGE);
  attachInterrupt(Interrupt_right_A, ai8, CHANGE);
  attachInterrupt(Interrupt_right_B, ai9, CHANGE);

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

    //rightPub.publish( &right_wheel_tick_count );
    //leftPub.publish( &left_wheel_tick_count );
    imupub.publish( &imu_data );
  }
  rightPub.publish( &right_wheel_tick_count );
  leftPub.publish( &left_wheel_tick_count );
  nh.spinOnce(); // "spinonce" have to be outside of "if" to avoid msg not publish
}

void ai10() {
  if (digitalRead(Pin_left_A) == HIGH) {
    if (digitalRead(Pin_left_B) == LOW) {
      counter_left--;
    }
    else {
      counter_left++;
    }
  }
  else {
    if (digitalRead(Pin_left_B) == LOW) {
      counter_left++;
    }
    else {
      counter_left--;
    }
  }
  left_wheel_tick_count.data = counter_left;
}

void ai11() {
  if (digitalRead(Pin_left_B) == HIGH) {
    if (digitalRead(Pin_left_A) == LOW) {
      counter_left++;
    }
    else {
      counter_left--;
    }
  }
  else {
    if (digitalRead(Pin_left_A) == LOW) {
      counter_left--;
    }
    else {
      counter_left++;
    }
  }
  left_wheel_tick_count.data = counter_left;
}


void ai8() {
  if (digitalRead(Pin_right_A) == HIGH) {
    if (digitalRead(Pin_right_B) == LOW) {
      counter_right++;
    }
    else {
      counter_right--;
    }
  }
  else {
    if (digitalRead(Pin_right_B) == LOW) {
      counter_right--;
    }
    else {
      counter_right++;
    }
  }
  right_wheel_tick_count.data = counter_right;
}

void ai9() {
  if (digitalRead(Pin_right_B) == HIGH) {
    if (digitalRead(Pin_right_A) == LOW) {
      counter_right--;
    }
    else {
      counter_right++;
    }
  }
  else {
    if (digitalRead(Pin_right_A) == LOW) {
      counter_right++;
    }
    else {
      counter_right--;
    }
  }
  right_wheel_tick_count.data = counter_right;
}
