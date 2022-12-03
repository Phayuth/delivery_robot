int pwm_cal (float u_in, float v_max) {
  int pwm = (u_in / v_max) * 250;
  // Limit PWM
  if (pwm < -250) {
    pwm = -250;
  }
  if (pwm > 250) {
    pwm = 250;
  }
  return pwm;
}

void L_write_pwm(int left_pwm)
{
  if ( left_pwm < 0) {
    analogWrite(L_pwm_pin, abs(left_pwm));
    digitalWrite(L_dir_pin, 0);
  }
  else if ( left_pwm > 0) {
    analogWrite(L_pwm_pin, left_pwm);
    digitalWrite(L_dir_pin, 1);
  }
  else {
    analogWrite(L_pwm_pin, 0);
    digitalWrite(L_dir_pin, 0);
  }
}

void R_write_pwm(int right_pwm)
{
  if (right_pwm < 0) {
    analogWrite(R_pwm_pin, abs(right_pwm));
    digitalWrite(R_dir_pin, 1);
  }
  else if (right_pwm > 0) {
    analogWrite(R_pwm_pin, right_pwm);
    digitalWrite(R_dir_pin, 0);
  }
  else {
    analogWrite(R_pwm_pin, 0);
    digitalWrite(R_dir_pin, 0);
  }
}
