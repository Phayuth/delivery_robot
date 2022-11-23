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
