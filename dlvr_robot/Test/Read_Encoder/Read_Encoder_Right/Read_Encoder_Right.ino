// Right Motor with forward positive count ( Clockwise )
#define motorR_Rpwm_pin 7
#define motorR_Lpwm_pin 6

int32_t R_counter = 0;
uint8_t R_PinA = 8;
uint8_t R_PinB = 9;
uint8_t R_InterruptA = 8;
uint8_t R_InterruptB = 9;

void setup() {
  Serial.begin(9600);
  pinMode(motorR_Rpwm_pin, OUTPUT);
  pinMode(motorR_Lpwm_pin, OUTPUT);

  pinMode(R_PinA, INPUT_PULLUP);
  pinMode(R_PinB, INPUT_PULLUP);
  attachInterrupt(R_InterruptA, ai8, CHANGE);
  attachInterrupt(R_InterruptB, ai9, CHANGE);
}

void loop() {
  analogWrite(motorR_Rpwm_pin, 100);
  analogWrite(motorR_Lpwm_pin, 0);

  Serial.print("Counter : ");
  Serial.print(R_counter);
  Serial.println("");
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
