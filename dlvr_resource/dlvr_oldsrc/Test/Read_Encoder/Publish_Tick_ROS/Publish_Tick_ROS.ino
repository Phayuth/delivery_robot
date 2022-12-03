#define USE_USBCON // For Arduino Due rosserial in every board and computer
#include <ros.h>
#include <std_msgs/Int16.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Encoder and Motor Left
#define left_motor_Rpwm_pin 5
#define left_motor_Lpwm_pin 4
int16_t counter_left = 0;
uint8_t Pin_left_A = 10;
uint8_t Pin_left_B = 11;
uint8_t Interrupt_left_A = 10;
uint8_t Interrupt_left_B = 11;

// Encoder and Motor Right
#define right_motor_Rpwm_pin 7
#define right_motor_Lpwm_pin 6
int16_t counter_right = 0;
uint8_t Pin_right_A = 8;
uint8_t Pin_right_B = 9;
uint8_t Interrupt_right_A = 8;
uint8_t Interrupt_right_B = 9;

// Minumum and maximum values for 16-bit integers
//const int encoder_minimum = -32768;
//const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

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

  // ROS Setup
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}

void loop() {
  analogWrite(left_motor_Rpwm_pin, 100);
  analogWrite(left_motor_Lpwm_pin, 0);
  analogWrite(right_motor_Rpwm_pin, 100);
  analogWrite(right_motor_Lpwm_pin, 0);


  // Record the time
  currentMillis = millis();

  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
  }
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
