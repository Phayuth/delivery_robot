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
