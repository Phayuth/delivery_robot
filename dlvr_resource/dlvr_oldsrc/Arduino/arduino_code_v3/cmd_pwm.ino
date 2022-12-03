// callback function for subcribe to left wheel
void left_cmd_pwm_cb(const std_msgs::Int32& msg)
{
  // get linear and angular velocity from twist
  left_pwm = msg.data;

  // write pwm to pin
  if ( left_pwm < 0) {
    analogWrite(left_motor_pwm, abs(left_pwm));
    digitalWrite(left_motor_dir, 0);
  }
  else if ( left_pwm > 0) {
    analogWrite(left_motor_pwm, left_pwm);
    digitalWrite(left_motor_dir, 1);
  }
  else {
    analogWrite(left_motor_pwm, 0);
    digitalWrite(left_motor_dir, 0);
  }
}


// callback function for subcribe to right wheel
void right_cmd_pwm_cb(const std_msgs::Int32& msg)
{
  // get linear and angular velocity from twist
  right_pwm = msg.data;

  // write pwm to pin
  if (right_pwm < 0) {
    analogWrite(right_motor_pwm, abs(right_pwm));
    digitalWrite(right_motor_dir, 1);
  }
  else if (right_pwm > 0) {
    analogWrite(right_motor_pwm, right_pwm);
    digitalWrite(right_motor_dir, 0);
  }
  else {
    analogWrite(right_motor_pwm, 0);
    digitalWrite(right_motor_dir, 0);
  }
}
