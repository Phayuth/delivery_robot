void ai10() {
  if (digitalRead(L_enc_A) == HIGH) {
    if (digitalRead(L_enc_B) == LOW) {
      L_counter--;
    }
    else {
      L_counter++;
    }
  }
  else {
    if (digitalRead(L_enc_B) == LOW) {
      L_counter++;
    }
    else {
      L_counter--;
    }
  }
}

void ai11() {
  if (digitalRead(L_enc_B) == HIGH) {
    if (digitalRead(L_enc_A) == LOW) {
      L_counter++;
    }
    else {
      L_counter--;
    }
  }
  else {
    if (digitalRead(L_enc_A) == LOW) {
      L_counter--;
    }
    else {
      L_counter++;
    }
  }
}


void ai8() {
  if (digitalRead(R_enc_A) == HIGH) {
    if (digitalRead(R_enc_B) == LOW) {
      R_counter++;
    }
    else {
      R_counter--;
    }
  }
  else {
    if (digitalRead(R_enc_B) == LOW) {
      R_counter--;
    }
    else {
      R_counter++;
    }
  }
}

void ai9() {
  if (digitalRead(R_enc_B) == HIGH) {
    if (digitalRead(R_enc_A) == LOW) {
      R_counter--;
    }
    else {
      R_counter++;
    }
  }
  else {
    if (digitalRead(R_enc_A) == LOW) {
      R_counter++;
    }
    else {
      R_counter--;
    }
  }
}
