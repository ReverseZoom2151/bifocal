#ifndef STEERING_H
#define STEERING_H

// Steering helpers for the line follower.
//
// Provides:
//   - linePosition5():  full 5-sensor weighted line-position error in [-1, 1].
//   - PID_c:            a tiny PID controller that turns a line-position error
//                       into a steering (turn) term.
//
// These operate on a plain float calibrated[5] array so they work for either
// the analog or the digital sensor pipeline without depending on the class
// type. Index 0 = left ... 4 = right, values normalised to ~0..1.

// Small constant to guard against divide-by-zero when no line is seen.
#define STEER_EPS 0.0001

// Full 5-sensor weighted line position.
//
// Standard weighted-average-of-index formula with symmetric weights
// { -2, -1, 0, 1, 2 }. The weighted sum divided by the total reflectance gives
// a position in [-2, 2], which is scaled to a normalised error in [-1, 1].
// Negative steers left, positive steers right. Returns 0.0 when no line is
// visible (total reflectance ~0), matching the old 2-sensor behaviour.
inline float linePosition5(float calibrated[5]) {

  const float weights[5] = { -2.0, -1.0, 0.0, 1.0, 2.0 };

  float weightedSum = 0.0;
  float total       = 0.0;
  for (int i = 0; i < 5; i++) {
    weightedSum += weights[i] * calibrated[i];
    total       += calibrated[i];
  }

  if (total < STEER_EPS) return 0.0;   // no line: report centred

  float position = weightedSum / total;   // range about [-2, 2]
  return position / 2.0;                   // normalise to [-1, 1]

}

// Old 2-sensor weighted measurement, kept available for comparison. Uses only
// the mid-left (1) and mid-right (3) sensors. W in [-1, 1].
inline float linePosition2(float calibrated[5]) {

  float sum = calibrated[1] + calibrated[3];
  if (sum == 0.0) return 0.0;
  float left  = calibrated[1] / sum;
  float right = calibrated[3] / sum;
  return right - left;

}

// Tiny PID controller operating on the line-position error.
//
// Produces a turn term from the error signal. The integral is clamped to stop
// wind-up, and the previous error is stored for the derivative term. Gains are
// deliberately small defaults and WILL need tuning on the real robot.
class PID_c {

  public:

    float Kp;
    float Ki;
    float Kd;
    float integralClamp;   // magnitude limit on the accumulated integral

    float integral;
    float lastError;

    PID_c() {
      // Sane starting gains. These are placeholders: tune Kp first, then Kd to
      // damp overshoot, then a little Ki only if a steady offset remains.
      Kp            = 1.0;
      Ki            = 0.0;
      Kd            = 0.2;
      integralClamp = 1.0;
      integral      = 0.0;
      lastError     = 0.0;
    }

    void reset() {
      integral  = 0.0;
      lastError = 0.0;
    }

    // Run one control step. dt is the loop period in seconds. Returns the turn
    // term (same sign convention as the error: negative left, positive right).
    float update(float error, float dt) {

      if (dt <= 0.0) dt = 0.0001;   // guard against a zero/negative period

      integral += error * dt;
      integral = constrain(integral, -integralClamp, integralClamp);

      float derivative = (error - lastError) / dt;
      lastError = error;

      return (Kp * error) + (Ki * integral) + (Kd * derivative);

    }

};

#endif
