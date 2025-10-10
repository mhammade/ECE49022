float angle = 0;
float step  = 0.1;   // speed → bigger step = higher f_elec
float amp   = 128;   // torque → bigger amp = higher current

void loop() {
  angle += step;
  if (angle > 2*PI) angle -= 2*PI;

  int duty = amp * sin(angle) + 128; // map sine to 0–255 (8-bit PWM)
  analogWrite(9, duty);              // send PWM on pin D9
  delayMicroseconds(50);             // controls update rate
}

