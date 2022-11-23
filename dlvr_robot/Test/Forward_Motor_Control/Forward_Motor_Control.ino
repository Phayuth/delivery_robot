// define motor pin
#define left_motor_Rpwm_pin 5
#define left_motor_Lpwm_pin 4
#define right_motor_Rpwm_pin 7
#define right_motor_Lpwm_pin 6

// preassign variable
int left_pwm;
int right_pwm;

// mobile robot parameter
float V;      // linear velocity
float omega;  // angular velocity
const float L = 0.44;  // robot base length in m
const float r = 0.07;  // robot wheel radius in m
float left_speed;
float right_speed;


void setup() {
  // set pins motor output
  pinMode(left_motor_Rpwm_pin, OUTPUT);
  pinMode(left_motor_Lpwm_pin, OUTPUT);
  pinMode(right_motor_Rpwm_pin, OUTPUT);
  pinMode(right_motor_Lpwm_pin, OUTPUT);
}

void loop() {

  // get V and omega from controller
  V = 0.2;
  omega = 0.01;

  // convert V and omega to each wheel speed
  left_speed = (2 * V - L * omega) / (2 * r);
  right_speed = (2 * V + L * omega) / (2 * r);

  // convert each wheel speed to pwm value, y=Ax+b, A=0.22,b=0.1
  left_pwm = 0.22 * left_speed + 0.1;
  right_pwm = 0.22 * right_speed + 0.1;
  
  // limit pwm
  if (left_pwm < -255){
    left_pwm = -255;
  }
  if (left_pwm > 255){
    left_pwm = 255;
  }
  if (right_pwm < -255){
    right_pwm = -255;
  }
  if (right_pwm > 255){
    right_pwm = 255;
  }

  // write pwm to pin
  if ( left_pwm < 0 && right_pwm < 0) {
    // left reverse , right reverse
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, left_pwm*-1);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, right_pwm*-1);
  }
  else if ( left_pwm > 0 && right_pwm > 0) {
    // left forward , right forward
    analogWrite(left_motor_Rpwm_pin, left_pwm);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, right_pwm);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
  else if ( left_pwm < 0 && right_pwm > 0) {
    // left reverse , right forward
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, left_pwm*-1);
    analogWrite(right_motor_Rpwm_pin, right_pwm);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
  else if ( left_pwm > 0 && right_pwm < 0) {
    // left forward , right reverse
    analogWrite(left_motor_Rpwm_pin, left_pwm);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, right_pwm*-1);
  }
  else{
    // neutral
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
}
