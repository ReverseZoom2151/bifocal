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
 *
 * Optional modules can be switched on with the flags in the configuration block
 * (all default off, baseline behavior unchanged): a 1D Kalman fusion filter
 * (kalman.h), a learned switching policy (policy.h), line-loss recovery
 * (recovery.h), and analog/digital disagreement logging (disagreement.h).
 */

#include "analoglinesensors.h"
#include "digitallinesensors.h"
#include "steering.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "persistence.h"
#include "kalman.h"
#include "disagreement.h"
#include "recovery.h"
#include "policy.h"

// ---- Configuration ---------------------------------------------------------
#define MAX_RESULTS 80              // number of logged control ticks per trial

#define METHOD_ANALOG    0
#define METHOD_DIGITAL   1
#define METHOD_SWITCHING 2
#define METHOD_FUSION    3
#define FOLLOW_METHOD    METHOD_SWITCHING   // <-- default strategy (runtime-selectable)

// Adaptive forward speed. Instead of a constant forward bias, the bias is
// scaled down as the steering demand grows so the robot slows for sharp
// corrections and curves, and runs faster on straights. MAX_BIAS_PWM is used
// when the turn term is near zero, MIN_BIAS_PWM when it is saturated. Both are
// runtime-adjustable via the 's' serial command (see the CLI block below).
#define MIN_BIAS_PWM 18.0
#define MAX_BIAS_PWM 30.0

// Load a previously saved calibration from EEPROM (if valid) instead of
// spinning to recalibrate at every boot. Set to 1 to opt in. Even when 0, a
// fresh spin calibration is still written to EEPROM so it can be reused later.
#define USE_SAVED_CALIBRATION 0

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

// ---- Optional advanced modules (opt-in) ------------------------------------
// Each defaults to 0, which keeps the baseline behavior byte-identical. Turn one
// on to route the corresponding module in. They are independent.
//   FUSION_USE_KALMAN: METHOD_FUSION blends with a 1D Kalman filter (kalman.h)
//                      instead of the plain inverse-variance weighting.
//   SWITCH_USE_POLICY: METHOD_SWITCHING picks the array with the learned
//                      logistic policy (policy.h) instead of the fixed margin.
//   ENABLE_RECOVERY:   line loss triggers coast/search recovery (recovery.h)
//                      instead of a hard stop.
//   LOG_DISAGREEMENT:  compute the analog/digital disagreement each tick
//                      (disagreement.h) and add a column to the CSV.
#define FUSION_USE_KALMAN 0
#define SWITCH_USE_POLICY 0
#define ENABLE_RECOVERY   0
#define LOG_DISAGREEMENT  0

// Nominal forward distance per control tick (mm), used only by the recovery
// coast/search limits when ENABLE_RECOVERY is on. Replace with a real odometry
// distance if you need accurate gap bridging.
#define RECOVERY_TICK_MM 5.0

// Set to 1 to run the calibration/inspection loop in setup() instead of a trial.
#define DEBUG_INSPECT 0

// End-of-line detection thresholds (sum of the two steering sensors).
#define ANALOG_STOP_SUM  0.4
#define DIGITAL_STOP_SUM 0.1

// ---- State machine ---------------------------------------------------------
#define STATE_RUNNING_TRIAL 0
#define STATE_TRIAL_DONE    1

AnalogLineSensors_c    a_sensors;
DigitalLineSensors_c   d_sensors;
Kinematics_c           kinematics;
Motors_c               motors;
PID_c                  pid;
KalmanLine1D           kalman;      // used only when FUSION_USE_KALMAN
DisagreementDetector_c disagree;    // used only when LOG_DISAGREEMENT
LineRecovery_c         recovery;    // used only when ENABLE_RECOVERY

unsigned long update_ts;
unsigned long trial_start_ts;
int   results_index;
int   state;

const float MaxTurnPWM = 20.0;   // max differential applied from steering term

// ---- Runtime-tunable state (adjustable over serial without recompiling) ----
// g_method defaults to the compile-time FOLLOW_METHOD but can be changed live
// with the 'm' command. g_min_bias/g_max_bias hold the adaptive-speed bounds.
int   g_method   = FOLLOW_METHOD;
float g_min_bias = MIN_BIAS_PWM;
float g_max_bias = MAX_BIAS_PWM;

// ---- Per-tick control outputs (also used by the CSV logger) ----------------
float g_line_error;   // steering error actually used this tick, [-1, 1]
float g_var_a;        // analog average variance this tick
float g_var_d;        // digital average variance this tick
int   g_active;       // array/method used: 0 analog, 1 digital, 2 fusion
float g_bias;         // forward bias actually applied this tick (logged)
float g_disagree;     // analog/digital disagreement this tick (LOG_DISAGREEMENT)

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
//
// Adaptive speed: the forward bias is interpolated linearly between g_max_bias
// (at W == 0, i.e. a straight) and g_min_bias (at |W| == 1, i.e. a hard
// correction), so the robot slows into curves and speeds up on straights. The
// result is clamped to the [min, max] band and stored in g_bias for logging.
// speedScale (default 1.0) multiplies the forward bias; the recovery state
// machine uses it to slow down while searching, and 0 to stop.
void applySteering(float W, float speedScale = 1.0) {
  W = constrain(W, -1.0, 1.0);

  float mag = fabs(W);   // 0 straight ... 1 full turn demand
  g_bias = g_max_bias - (g_max_bias - g_min_bias) * mag;
  g_bias = constrain(g_bias, g_min_bias, g_max_bias);
  g_bias *= speedScale;

  float LeftPWM  = g_bias + (MaxTurnPWM * W);
  float RightPWM = g_bias - (MaxTurnPWM * W);
  motors.setMotorsPWM(LeftPWM, RightPWM);
}

// ---- Unified control step --------------------------------------------------
// Samples both arrays every tick so every log row is complete and the two
// methods stay directly comparable. Computes each array's line-position error
// and variance, selects/fuses per the runtime g_method, stores logging globals,
// and returns the turn term W.
float computeControl() {

  // calculateAverageVariance() refreshes calibrated[] as a side effect, so the
  // error can be read straight from it afterwards.
  g_var_a = a_sensors.calculateAverageVariance();
  float e_a = lineError(a_sensors.calibrated);

  g_var_d = d_sensors.calculateAverageVariance();
  float e_d = lineError(d_sensors.calibrated);

#if LOG_DISAGREEMENT
  // Track how much the two arrays disagree. A spike relative to the rolling
  // baseline suggests a line edge, junction, or surface anomaly.
  g_disagree = disagree.update(a_sensors.calibrated, d_sensors.calibrated);
#endif

  // Strategy selection is a runtime choice (g_method) so it can be changed live
  // over serial. It defaults to the compile-time FOLLOW_METHOD.
  if (g_method == METHOD_ANALOG) {
    g_active     = 0;
    g_line_error = e_a;

  } else if (g_method == METHOD_DIGITAL) {
    g_active     = 1;
    g_line_error = e_d;

  } else if (g_method == METHOD_FUSION) {
#if FUSION_USE_KALMAN
    // Bayesian fusion: each array's live variance is its measurement noise R.
    g_line_error = kalman.fuse(e_a, g_var_a, e_d, g_var_d);
#else
    // Inverse-variance blend: trust the steadier array more, without a hard pick.
    float w_a = 1.0 / (g_var_a + FUSION_EPS);
    float w_d = 1.0 / (g_var_d + FUSION_EPS);
    g_line_error = (w_a * e_a + w_d * e_d) / (w_a + w_d);
#endif
    g_active     = 2;

  } else {   // METHOD_SWITCHING with hysteresis + dwell
    // Only consider switching once the minimum dwell time has elapsed.
    if (millis() - switch_last_ts >= SWITCH_DWELL_MS) {
#if SWITCH_USE_POLICY
      // Learned logistic policy decides which array to trust from the two
      // variances and the current error (scale-invariant, see policy.h).
      int want = preferDigital(g_var_a, g_var_d, g_line_error) ? 1 : 0;
      if (want != switch_choice) {
        switch_choice  = want;
        switch_last_ts = millis();
      }
#else
      // Fixed margin: switch only if the alternative is better by the margin.
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
#endif
    }
    g_active     = switch_choice;
    g_line_error = (switch_choice == 0) ? e_a : e_d;
  }

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

  // End-of-line detection on the two steering sensors of the relevant array(s).
  float a_sum = a_sensors.calibrated[1] + a_sensors.calibrated[3];
  float d_sum = d_sensors.calibrated[1] + d_sensors.calibrated[3];

#if ENABLE_RECOVERY
  // Instead of a hard stop when the line is lost, coast a short distance to
  // bridge small gaps, then sweep toward the last-known side to reacquire.
  bool linePresent = (a_sum >= ANALOG_STOP_SUM) || (d_sum >= DIGITAL_STOP_SUM);
  RecoveryAction act = recovery.update(linePresent, g_line_error,
                                       RECOVERY_TICK_MM, (float)CONTROL_PERIOD_MS);
  if (act.state == REC_STOPPED || act.speedScale <= 0.0) {
    motors.setMotorsPWM(0, 0);
  } else if (act.useNormalSteering) {
    applySteering(W, act.speedScale);
  } else {
    applySteering(act.turnBias, act.speedScale);   // sweep to reacquire
  }
#else
  applySteering(W);

  // End-of-line test follows the active runtime method.
  if (g_method == METHOD_ANALOG) {
    if (a_sum < ANALOG_STOP_SUM) motors.setMotorsPWM(0, 0);
  } else if (g_method == METHOD_DIGITAL) {
    if (d_sum < DIGITAL_STOP_SUM) motors.setMotorsPWM(0, 0);
  } else {
    // Switching and fusion both listen to the whole board: stop only when both
    // arrays have lost the line.
    if (a_sum < ANALOG_STOP_SUM && d_sum < DIGITAL_STOP_SUM) {
      motors.setMotorsPWM(0, 0);
    }
  }
#endif

}

// ---- Logging ---------------------------------------------------------------
// Clean, fixed-rate, timestamped CSV. One header line, then one row per control
// tick. No growing history is reprinted, so any serial capture (not just PuTTY)
// yields a directly loadable CSV. Collecting more and longer runs is now just a
// hardware step: this logging is what makes those runs usable.
void printLogHeader() {
#if LOG_DISAGREEMENT
  Serial.println("t_ms,theta,method,line_error,var_a,var_d,bias,disagree");
#else
  Serial.println("t_ms,theta,method,line_error,var_a,var_d,bias");
#endif
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
  Serial.print(g_var_d, 6);
  Serial.print(",");
#if LOG_DISAGREEMENT
  Serial.print(g_bias, 2);
  Serial.print(",");
  Serial.println(g_disagree, 4);
#else
  Serial.println(g_bias, 2);
#endif
}

// ---- EEPROM calibration helpers --------------------------------------------
// Read the live calibration out of both sensor objects and persist it.
void saveCurrentCalibration() {
  float aOff[5], aScale[5], dOff[5], dScale[5];
  a_sensors.getCalibration(aOff, aScale);
  d_sensors.getCalibration(dOff, dScale);
  saveCalibration(aOff, aScale, dOff, dScale);
}

// Load a stored calibration (if valid) into both sensor objects. Returns false
// and changes nothing when no valid block is present.
bool loadStoredCalibration() {
  float aOff[5], aScale[5], dOff[5], dScale[5];
  if (!loadCalibration(aOff, aScale, dOff, dScale)) return false;
  a_sensors.setCalibration(aOff, aScale);
  d_sensors.setCalibration(dOff, dScale);
  return true;
}

// ---- Live serial tuning CLI ------------------------------------------------
// A tiny, non-blocking, fixed-buffer command parser so parameters can be
// changed at runtime without recompiling. Lines are terminated by newline.
// Commands (arguments are whitespace separated):
//   m <0-3>            select method: 0 analog, 1 digital, 2 switching, 3 fusion
//   p <kp> <ki> <kd>   set PID gains (also resets the integrator)
//   s <min> <max>      set adaptive-speed bias bounds (0 <= min <= max)
//   w                  write/save the current calibration to EEPROM
//   l                  load the saved calibration from EEPROM
//   c                  clear (invalidate) the saved calibration
//   ?                  print current settings
// Responses starting with '#' are informational, not CSV rows.
#define CMD_BUF_LEN 48
char cmd_buf[CMD_BUF_LEN];
int  cmd_len = 0;

void printSettings() {
  Serial.print("# method=");
  Serial.println(g_method);
  Serial.print("# kp=");
  Serial.print(pid.Kp, 4);
  Serial.print(" ki=");
  Serial.print(pid.Ki, 4);
  Serial.print(" kd=");
  Serial.println(pid.Kd, 4);
  Serial.print("# min_bias=");
  Serial.print(g_min_bias, 2);
  Serial.print(" max_bias=");
  Serial.println(g_max_bias, 2);
  Serial.print("# saved_cal=");
  Serial.println(hasValidCalibration() ? 1 : 0);
}

// Parse and act on one complete command line. Uses strtok/atof only; no String
// and no dynamic allocation.
void handleCommand(char *line) {
  char *tok = strtok(line, " \t");
  if (tok == NULL) return;
  char cmd = tok[0];

  switch (cmd) {
    case 'm': {
      char *a = strtok(NULL, " \t");
      if (a != NULL) {
        int m = atoi(a);
        if (m >= METHOD_ANALOG && m <= METHOD_FUSION) {
          g_method = m;
          Serial.print("# method set to ");
          Serial.println(g_method);
        } else {
          Serial.println("# err: method must be 0-3");
        }
      } else {
        Serial.println("# err: m <0-3>");
      }
      break;
    }

    case 'p': {
      char *a = strtok(NULL, " \t");
      char *b = strtok(NULL, " \t");
      char *c = strtok(NULL, " \t");
      if (a != NULL && b != NULL && c != NULL) {
        pid.Kp = atof(a);
        pid.Ki = atof(b);
        pid.Kd = atof(c);
        pid.reset();
        Serial.println("# pid gains updated");
      } else {
        Serial.println("# err: p <kp> <ki> <kd>");
      }
      break;
    }

    case 's': {
      char *a = strtok(NULL, " \t");
      char *b = strtok(NULL, " \t");
      if (a != NULL && b != NULL) {
        float mn = atof(a);
        float mx = atof(b);
        if (mn >= 0.0 && mx >= mn) {
          g_min_bias = mn;
          g_max_bias = mx;
          Serial.println("# speed bounds updated");
        } else {
          Serial.println("# err: need 0 <= min <= max");
        }
      } else {
        Serial.println("# err: s <min> <max>");
      }
      break;
    }

    case 'w':
      saveCurrentCalibration();
      Serial.println("# calibration saved");
      break;

    case 'l':
      if (loadStoredCalibration()) {
        Serial.println("# calibration loaded");
      } else {
        Serial.println("# err: no valid saved calibration");
      }
      break;

    case 'c':
      clearCalibration();
      Serial.println("# calibration cleared");
      break;

    case '?':
      printSettings();
      break;

    default:
      Serial.println("# err: unknown cmd");
      break;
  }
}

// Non-blocking serial reader. Accumulates characters into a fixed buffer and
// dispatches on newline. Safe to call every loop iteration.
void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;             // ignore CR (CRLF terminals)
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len > 0) handleCommand(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len < CMD_BUF_LEN - 1) {
      cmd_buf[cmd_len++] = c;
    } else {
      cmd_len = 0;                        // overflow: drop the partial line
    }
  }
}

// ---- Arduino entry points --------------------------------------------------
void setup() {

  Serial.begin(9600);
  delay(2000);
  Serial.println("***RESET***");

  motors.initialise();      // configure motor pins BEFORE driving
  setupEncoder0();
  setupEncoder1();

  // Calibration: optionally reuse a valid EEPROM-stored calibration and skip
  // the spin, otherwise spin on the spot over black + white and store the new
  // calibration for future boots. The magic guard means a blank EEPROM never
  // loads garbage.
  bool loaded = false;
#if USE_SAVED_CALIBRATION
  if (hasValidCalibration()) {
    loaded = loadStoredCalibration();
    Serial.println(loaded ? "# loaded saved calibration"
                          : "# saved calibration load failed");
  }
#endif

  if (!loaded) {
    motors.setMotorsPWM(-80, 80);
    a_sensors.calibrate();
    d_sensors.calibrate();
    motors.setMotorsPWM(0, 0);
    saveCurrentCalibration();
    Serial.println("# calibration saved to EEPROM");
  }

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

  // Always service the serial CLI, in every state, so parameters can be tuned
  // during a trial or after it has finished.
  pollSerial();

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
