void left_cmd_pwm_cb(const std_msgs::Float32& msg)
{
  L_omega_d = msg.data;
}

void right_cmd_pwm_cb(const std_msgs::Float32& msg)
{
  R_omega_d = msg.data;
}

float tick_2_theta(int tick, float PPR, float GR) {
  float theta = (2 * pi * tick) / (PPR * GR);
  return theta;
}
