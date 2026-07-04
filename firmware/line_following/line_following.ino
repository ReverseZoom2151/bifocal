/*
 * Variance-Based Sensor Switching -- Line-Following Robot
 * -------------------------------------------------------
 * Pololu 3Pi+ / Romi-class robot. Follows a line using one of four strategies,
 * selectable below via FOLLOW_METHOD:
 *
 *   METHOD_ANALOG    - always use the analog IR array
 *   METHOD_DIGITAL   - always use the digital (RC decay) IR array
 *   METHOD_SWITCHING - dynamically pick whichever array currently has the
 *                      lower average reading variance (the novel approach),
 *                      debounced with hysteresis and a minimum dwell time
 *   METHOD_FUSION    - blend the two arrays' line-position errors by
 *                      inverse-variance weighting (no hard pick)
 *
 * Steering uses a full 5-sensor weighted line position by default (weights
 * -2..2), with the old 2-sensor measurement still available behind a flag. The
 * turn term is either a simple proportional term or a small PID (STEER_MODE).
 * Wheel-encoder odometry logs heading (theta). Every control tick is written as
 * one timestamped CSV row over Serial for offline analysis.
 */

#include "analoglinesensors.h"
#include "digitallinesensors.h"
#include "steering.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"

// ---- Configuration ---------------------------------------------------------
#define MAX_RESULTS 80              // number of logged control ticks per trial

#define METHOD_ANALOG    0
#define METHOD_DIGITAL   1
#define METHOD_SWITCHING 2
#define METHOD_FUSION    3
#define FOLLOW_METHOD    METHOD_SWITCHING   // <-- choose the strategy here

// Steering input: 1 uses the full 5-sensor weighted position (default),
// 0 falls back to the old 2-sensor measurement (sensors 1 and 3 only).
#define USE_FULL_POSITION 1

// Turn-term controller: STEER_SIMPLE is plain proportional (turn = error),
// STEER_PID runs the PID_c controller from steering.h.
#define STEER_SIMPLE 0
#define STEER_PID    1
#define STEER_MODE   STEER_PID

// Switch debounce (METHOD_SWITCHING only). Only switch arrays if the
// alternative's average variance is lower by at least SWITCH_MARGIN, and never
// switch more often than SWITCH_DWELL_MS. Together these stop the per-tick
// flip-flopping that adds heading wobble.
#define SWITCH_MARGIN   0.0005
#define SWITCH_DWELL_MS 300

// Inverse-variance fusion guard (METHOD_FUSION only). Added to each variance
// before inverting so a zero variance cannot produce an infinite weight.
#define FUSION_EPS 0.0001

// Control loop timing.
#define CONTROL_PERIOD_MS 100      // run the controller at ~10 Hz
#define CONTROL_DT        0.1      // matching period in seconds, for PID

// Set to 1 to run the calibration/inspection loop in setup() instead of a trial.
#define DEBUG_INSPECT 0

// End-of-line detection thresholds (sum of the two steering sensors).
#define ANALOG_STOP_SUM  0.4
#define DIGITAL_STOP_SUM 0.1

// ---- State machine ---------------------------------------------------------
#define STATE_RUNNING_TRIAL 0
#define STATE_TRIAL_DONE    1

AnalogLineSensors_c  a_sensors;
DigitalLineSensors_c d_sensors;
Kinematics_c         kinematics;
Motors_c             motors;
PID_c                pid;

unsigned long update_ts;
unsigned long trial_start_ts;
int   results_index;
int   state;

const float BiasPWM    = 30.0;   // forward bias
const float MaxTurnPWM = 20.0;   // max differential applied from steering term

// ---- Per-tick control outputs (also used by the CSV logger) ----------------
float g_line_error;   // steering error actually used this tick, [-1, 1]
float g_var_a;        // analog average variance this tick
float g_var_d;        // digital average variance this tick
int   g_active;       // array/method used: 0 analog, 1 digital, 2 fusion

// Switching debounce state.
int           switch_choice = 0;   // 0 analog, 1 digital
unsigned long switch_last_ts = 0;

// ---- Line-position helpers -------------------------------------------------
// Dispatch between the full 5-sensor position and the old 2-sensor one.
float lineError(float calibrated[5]) {
#if USE_FULL_POSITION
  return linePosition5(calibrated);
#else
  return linePosition2(calibrated);
#endif
}

// Convert a steering term into motor commands and apply them. W is clamped to
// [-1, 1] so a large PID output cannot invert or saturate the drive.
void applySteering(float W) {
  W = constrain(W, -1.0, 1.0);
  float LeftPWM  = BiasPWM + (MaxTurnPWM * W);
  float RightPWM = BiasPWM - (MaxTurnPWM * W);
  motors.setMotorsPWM(LeftPWM, RightPWM);
}

// ---- Unified control step --------------------------------------------------
// Samples both arrays every tick so every log row is complete and the two
// methods stay directly comparable. Computes each array's line-position error
// and variance, selects/fuses per FOLLOW_METHOD, stores logging globals, and
// returns the turn term W.
float computeControl() {

  // calculateAverageVariance() refreshes calibrated[] as a side effect, so the
  // error can be read straight from it afterwards.
  g_var_a = a_sensors.calculateAverageVariance();
  float e_a = lineError(a_sensors.calibrated);

  g_var_d = d_sensors.calculateAverageVariance();
  float e_d = lineError(d_sensors.calibrated);

#if FOLLOW_METHOD == METHOD_ANALOG
  g_active     = 0;
  g_line_error = e_a;

#elif FOLLOW_METHOD == METHOD_DIGITAL
  g_active     = 1;
  g_line_error = e_d;

#elif FOLLOW_METHOD == METHOD_FUSION
  // Inverse-variance blend: trust the steadier array more, without a hard pick.
  float w_a = 1.0 / (g_var_a + FUSION_EPS);
  float w_d = 1.0 / (g_var_d + FUSION_EPS);
  g_line_error = (w_a * e_a + w_d * e_d) / (w_a + w_d);
  g_active     = 2;

#else   // METHOD_SWITCHING with hysteresis + dwell
  // Only consider switching once the minimum dwell time has elapsed, and only
  // switch if the alternative is better by at least the margin.
  if (millis() - switch_last_ts >= SWITCH_DWELL_MS) {
    if (switch_choice == 0) {
      if (g_var_d + SWITCH_MARGIN < g_var_a) {
        switch_choice  = 1;
        switch_last_ts = millis();
      }
    } else {
      if (g_var_a + SWITCH_MARGIN < g_var_d) {
        switch_choice  = 0;
        switch_last_ts = millis();
      }
    }
  }
  g_active     = switch_choice;
  g_line_error = (switch_choice == 0) ? e_a : e_d;
#endif

#if STEER_MODE == STEER_PID
  return pid.update(g_line_error, CONTROL_DT);
#else
  return g_line_error;   // simple proportional: turn term is the error itself
#endif

}

// Run one control tick: odometry, steering, and end-of-line detection.
void followLine() {

  kinematics.update(count_e0, count_e1);

  float W = computeControl();
  applySteering(W);

  // End-of-line detection on the two steering sensors of the relevant array(s).
  float a_sum = a_sensors.calibrated[1] + a_sensors.calibrated[3];
  float d_sum = d_sensors.calibrated[1] + d_sensors.calibrated[3];

#if FOLLOW_METHOD == METHOD_ANALOG
  if (a_sum < ANALOG_STOP_SUM) motors.setMotorsPWM(0, 0);
#elif FOLLOW_METHOD == METHOD_DIGITAL
  if (d_sum < DIGITAL_STOP_SUM) motors.setMotorsPWM(0, 0);
#else
  // Switching and fusion both listen to the whole board: stop only when both
  // arrays have lost the line.
  if (a_sum < ANALOG_STOP_SUM && d_sum < DIGITAL_STOP_SUM) {
    motors.setMotorsPWM(0, 0);
  }
#endif

}

// ---- Logging ---------------------------------------------------------------
// Clean, fixed-rate, timestamped CSV. One header line, then one row per control
// tick. No growing history is reprinted, so any serial capture (not just PuTTY)
// yields a directly loadable CSV. Collecting more and longer runs is now just a
// hardware step: this logging is what makes those runs usable.
void printLogHeader() {
  Serial.println("t_ms,theta,method,line_error,var_a,var_d");
}

void logRow() {
  Serial.print(millis() - trial_start_ts);
  Serial.print(",");
  Serial.print(kinematics.theta, 4);
  Serial.print(",");
  Serial.print(g_active);
  Serial.print(",");
  Serial.print(g_line_error, 4);
  Serial.print(",");
  Serial.print(g_var_a, 6);
  Serial.print(",");
  Serial.println(g_var_d, 6);
}

// ---- Arduino entry points --------------------------------------------------
void setup() {

  Serial.begin(9600);
  delay(2000);
  Serial.println("***RESET***");

  motors.initialise();      // configure motor pins BEFORE driving
  setupEncoder0();
  setupEncoder1();

  // Calibrate by spinning on the spot over black + white surfaces.
  motors.setMotorsPWM(-80, 80);
  a_sensors.calibrate();
  d_sensors.calibrate();
  motors.setMotorsPWM(0, 0);

  // Beep + delay so the robot can be placed at the start line.
  pinMode(6, OUTPUT);
  for (int count = 0; count < 10; count++) {
    analogWrite(6, 120);
    delay(5);
    analogWrite(6, 0);
    delay(500);
  }

#if DEBUG_INSPECT
  // Optional: stream calibrated readings / variance for tuning. Never returns.
  while (true) {
    a_sensors.getCalibrated();
    a_sensors.printCalibrated();
    d_sensors.getCalibrated();
    d_sensors.printCalibrated();
    delay(10);
  }
#endif

  pid.reset();

  printLogHeader();

  update_ts      = millis();
  trial_start_ts = millis();
  results_index  = 0;
  state          = STATE_RUNNING_TRIAL;
}

void loop() {

  if (state == STATE_RUNNING_TRIAL) {

    if (millis() - update_ts > CONTROL_PERIOD_MS) {
      update_ts = millis();

      followLine();
      logRow();

      results_index++;
      if (results_index >= MAX_RESULTS) {
        // Trial complete: stop and signal. Nothing more is printed.
        state = STATE_TRIAL_DONE;
        motors.setMotorsPWM(0, 0);
        analogWrite(6, 120);
        delay(5);
        analogWrite(6, 0);
      }
    }

  }
  // STATE_TRIAL_DONE: idle. The CSV was streamed live, so there is no history
  // to reprint here.

}
