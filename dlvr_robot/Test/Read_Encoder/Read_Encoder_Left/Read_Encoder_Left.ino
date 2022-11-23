// Left Motor with forward positive count ( L_counter Clockwise )
#define motorL_Rpwm_pin 5
#define motorL_Lpwm_pin 4

int32_t L_counter = 0;
uint8_t L_PinA = 10;
uint8_t L_PinB = 11;
uint8_t L_InterruptA = 10;
uint8_t L_InterruptB = 11;

void setup() {
  Serial.begin(9600);
  pinMode(motorL_Rpwm_pin, OUTPUT);
  pinMode(motorL_Lpwm_pin, OUTPUT);

  pinMode(L_PinA, INPUT_PULLUP);
  pinMode(L_PinB, INPUT_PULLUP);
  attachInterrupt(L_InterruptA, ai10, CHANGE);
  attachInterrupt(L_InterruptB, ai11, CHANGE);
}

void loop() {
  analogWrite(motorL_Rpwm_pin, 100);
  analogWrite(motorL_Lpwm_pin, 0);

  Serial.print("L_counter : ");
  Serial.print(L_counter);
  Serial.println("");
}

void ai10() {
  if (digitalRead(L_PinA) == HIGH) {
    if (digitalRead(L_PinB) == LOW) {
      L_counter--;
    }
    else {
      L_counter++;
    }
  }
  else {
    if (digitalRead(L_PinB) == LOW) {
      L_counter++;
    }
    else {
      L_counter--;
    }
  }
}

void ai11() {
  if (digitalRead(L_PinB) == HIGH) {
    if (digitalRead(L_PinA) == LOW) {
      L_counter++;
    }
    else {
      L_counter--;
    }
  }
  else {
    if (digitalRead(L_PinA) == LOW) {
      L_counter--;
    }
    else {
      L_counter++;
    }
  }
}
