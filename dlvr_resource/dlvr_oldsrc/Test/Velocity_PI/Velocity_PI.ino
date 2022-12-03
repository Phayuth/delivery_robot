// Right Motor with forward positive count ( Clockwise )
#define motorR_Rpwm_pin 7
#define motorR_Lpwm_pin 6

// motor
float pi = 3.141592653;
float a = 27.65, b = 17.35; //from lumped parameter estimation
float PPR = 28, GR = 72; // Pulse per rev and gear box
float theta_p = 0, theta_c = 0;
float omega_d = 0, omega_d_p = 0, omega_c = 0;

//PID constants
float kp = (2 * 1 * 2 * pi * 6 - a) / b, ki = (2 * pi * 6 * 2 * pi * 6) / b;
float cumError = 0, error = 0;
float u = 0;
int pwm;

// time
unsigned long Time_old, Time_new, Time_change;
float Ts;

int R_counter = 0;
uint8_t R_PinA = 8;
uint8_t R_PinB = 9;
uint8_t R_InterruptA = 8;
uint8_t R_InterruptB = 9;

void setup() {
  pinMode(motorR_Rpwm_pin, OUTPUT);
  pinMode(motorR_Lpwm_pin, OUTPUT);
  Time_old = millis();
  Time_new = millis();
  Time_change = millis();
  pinMode(R_PinA, INPUT_PULLUP);
  pinMode(R_PinB, INPUT_PULLUP);
  attachInterrupt(R_InterruptA, ai8, CHANGE);
  attachInterrupt(R_InterruptB, ai9, CHANGE);
  omega_d = 3;
  Serial.begin(9600);
}

void loop() {
  // Find out how time has passed
  delay(1); // put delay here to avoid 0/0 NaN, I dont know how to fix it yet.
  Time_new = millis();
  Ts = (Time_new - Time_old) * 0.001; // convert from milli second (ms) to second(s)

  // Get reference input

  if (Time_new - Time_change > 5000) {
    omega_d = 9;
  }

  if (Time_new - Time_change > 10000) {
    omega_d = 5;
  }
  // Find current theta and find omega
  theta_c = tick_2_theta(R_counter, PPR, GR);
  omega_c = (theta_c - theta_p) / Ts;

  // Calculate different error
  error = omega_d - omega_c;

  // Calculate input with PI and compensation ( PI for velocity controller using 2nd ODE standard form )
  u = kp * error + ki * (cumError + error * Ts) + (a / b) * omega_d + (1 / b) * ((omega_d - omega_d_p) / Ts);
  Serial.println(omega_c);


  // Write control value to motor
  pwm = pwm_cal(u, 23.5);
  write_pwm(pwm);
  
  // Update for next time step
  Time_old = Time_new;
  theta_p = theta_c;
  cumError += error * Ts; // integral for error accumulate
  omega_d_p = omega_d;
}

void ai8() {
  if (digitalRead(R_PinA) == HIGH) {
    if (digitalRead(R_PinB) == LOW) {
      R_counter++;
    }
    else {
      R_counter--;
    }
  }
  else {
    if (digitalRead(R_PinB) == LOW) {
      R_counter--;
    }
    else {
      R_counter++;
    }
  }
}

void ai9() {
  if (digitalRead(R_PinB) == HIGH) {
    if (digitalRead(R_PinA) == LOW) {
      R_counter--;
    }
    else {
      R_counter++;
    }
  }
  else {
    if (digitalRead(R_PinA) == LOW) {
      R_counter++;
    }
    else {
      R_counter--;
    }
  }
}

float tick_2_theta(int tick, float PPR, float GR) {
  float theta = (2 * pi * tick) / (PPR * GR);
  return theta;
}

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

void write_pwm(int pwm) {
  if ( pwm < 0) {
    analogWrite(motorR_Rpwm_pin, 0);
    analogWrite(motorR_Lpwm_pin, pwm * -1);
  }
  else if ( pwm > 0) {
    analogWrite(motorR_Rpwm_pin, pwm);
    analogWrite(motorR_Lpwm_pin, 0);
  }
  else {
    analogWrite(motorR_Rpwm_pin, 0);
    analogWrite(motorR_Lpwm_pin, 0);
  }
}
