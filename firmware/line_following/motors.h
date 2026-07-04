#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM 10
#define L_DIR 16
#define R_PWM  9
#define R_DIR 15
#define FWD LOW
#define BCK HIGH

// Controls the two drive motors via PWM + direction pins.
class Motors_c {

  public:

    Motors_c() {
      // Empty on purpose: call initialise() from setup() before driving.
    }

    // Configure motor pins and start stopped. MUST be called in setup().
    void initialise() {

      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      digitalWrite(L_DIR, FWD);
      digitalWrite(R_DIR, FWD);

      setMotorsPWM(0, 0);   // ensure we start halted
    }

    // Signed PWM: sign selects direction, magnitude is clamped to 0..255.
    void setMotorsPWM(float left, float right) {

      digitalWrite(L_DIR, (left  < 0) ? BCK : FWD);
      digitalWrite(R_DIR, (right < 0) ? BCK : FWD);

      left  = constrain(abs(left),  0.0, 255.0);
      right = constrain(abs(right), 0.0, 255.0);

      analogWrite(L_PWM, (int)left);
      analogWrite(R_PWM, (int)right);
    }

    void driveStraight(int speed) {
      digitalWrite(L_DIR, FWD);
      digitalWrite(R_DIR, FWD);
      analogWrite(L_PWM, speed);
      analogWrite(R_PWM, speed);
    }

    void spinLeft(const float speed, int pause) {
      digitalWrite(L_DIR, BCK);
      digitalWrite(R_DIR, FWD);
      analogWrite(L_PWM, speed);
      analogWrite(R_PWM, speed);
      delay(pause);
    }

    void spinRight(const float speed, int pause) {
      digitalWrite(L_DIR, FWD);
      digitalWrite(R_DIR, BCK);
      analogWrite(L_PWM, speed);
      analogWrite(R_PWM, speed);
      delay(pause);
    }

};

#endif
