/*
 * Variance-Based Sensor Switching -- Line-Following Robot
 * -------------------------------------------------------
 * Pololu 3Pi+ / Romi-class robot. Follows a line using one of three strategies,
 * selectable below via FOLLOW_METHOD:
 *
 *   METHOD_ANALOG    - always use the analog IR array
 *   METHOD_DIGITAL   - always use the digital (RC decay) IR array
 *   METHOD_SWITCHING - dynamically pick whichever array currently has the
 *                      lower average reading variance (the novel approach)
 *
 * Steering uses a weighted measurement W = right - left from the mid-left (index
 * 1) and mid-right (index 3) sensors, applied as a differential to a forward
 * bias PWM. Wheel-encoder odometry logs heading (theta) for offline analysis.
 */

#include "analoglinesensors.h"
#include "digitallinesensors.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"

// ---- Configuration ---------------------------------------------------------
#define MAX_RESULTS 80

#define METHOD_ANALOG    0
#define METHOD_DIGITAL   1
#define METHOD_SWITCHING 2
#define FOLLOW_METHOD    METHOD_SWITCHING   // <-- choose the strategy here

// Set to 1 to run the calibration/inspection loop in setup() instead of a trial.
#define DEBUG_INSPECT 0

// End-of-line detection thresholds (sum of the two steering sensors).
#define ANALOG_STOP_SUM  0.4
#define DIGITAL_STOP_SUM 0.1

// ---- State machine ---------------------------------------------------------
#define STATE_RUNNING_TRIAL 0
#define STATE_PRINT_RESULTS 1

AnalogLineSensors_c  a_sensors;
DigitalLineSensors_c d_sensors;
Kinematics_c         kinematics;
Motors_c             motors;

unsigned long update_ts;
float results[MAX_RESULTS];   // logged theta samples for this trial
int   results_index;
int   state;

const float BiasPWM    = 30.0;   // forward bias
const float MaxTurnPWM = 20.0;   // max differential applied from steering term

// ---- Weighted steering measurements ---------------------------------------
// W in [-1, 1]: negative steers left, positive steers right.
float analogWeightedMeasurement() {
  a_sensors.getCalibrated();
  float sum = a_sensors.calibrated[1] + a_sensors.calibrated[3];
  if (sum == 0.0) return 0.0;
  float left  = a_sensors.calibrated[1] / sum;
  float right = a_sensors.calibrated[3] / sum;
  return right - left;
}

float digitalWeightedMeasurement() {
  d_sensors.getCalibrated();
  float sum = d_sensors.calibrated[1] + d_sensors.calibrated[3];
  if (sum == 0.0) return 0.0;
  float left  = d_sensors.calibrated[1] / sum;
  float right = d_sensors.calibrated[3] / sum;
  return right - left;
}

// Convert a steering term into motor commands and apply them.
void applySteering(float W) {
  float LeftPWM  = BiasPWM + (MaxTurnPWM * W);
  float RightPWM = BiasPWM - (MaxTurnPWM * W);
  motors.setMotorsPWM(LeftPWM, RightPWM);
}

// ---- Line-following strategies ---------------------------------------------
void analogFollowLine() {
  kinematics.update(count_e0, count_e1);
  applySteering(analogWeightedMeasurement());

  float sum = a_sensors.calibrated[1] + a_sensors.calibrated[3];
  if (sum < ANALOG_STOP_SUM) motors.setMotorsPWM(0, 0);   // end of line
}

void digitalFollowLine() {
  kinematics.update(count_e0, count_e1);
  applySteering(digitalWeightedMeasurement());

  float sum = d_sensors.calibrated[1] + d_sensors.calibrated[3];
  if (sum < DIGITAL_STOP_SUM) motors.setMotorsPWM(0, 0);  // end of line
}

// Variance-based switching: use whichever array is currently more stable.
void switchingFollowLine() {
  kinematics.update(count_e0, count_e1);

  float analogVariance  = a_sensors.calculateAverageVariance();
  float digitalVariance = d_sensors.calculateAverageVariance();

  float W;
  if (analogVariance < digitalVariance) {
    W = analogWeightedMeasurement();
  } else {
    W = digitalWeightedMeasurement();
  }
  applySteering(W);

  float a_sum = a_sensors.calibrated[1] + a_sensors.calibrated[3];
  float d_sum = d_sensors.calibrated[1] + d_sensors.calibrated[3];
  if (a_sum < ANALOG_STOP_SUM && d_sum < DIGITAL_STOP_SUM) {
    motors.setMotorsPWM(0, 0);   // end of line on both arrays
  }
}

// Dispatch to the configured strategy.
void followLine() {
#if   FOLLOW_METHOD == METHOD_ANALOG
  analogFollowLine();
#elif FOLLOW_METHOD == METHOD_DIGITAL
  digitalFollowLine();
#else
  switchingFollowLine();
#endif
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

  update_ts     = millis();
  results_index = 0;
  state         = STATE_RUNNING_TRIAL;
}

void loop() {

  if (state == STATE_RUNNING_TRIAL) {

    if (millis() - update_ts > 100) {   // run the controller at ~10 Hz
      update_ts = millis();

      followLine();

      if (results_index < MAX_RESULTS) {
        results[results_index] = kinematics.theta;   // log heading
        results_index++;
      } else {
        // Trial complete: stop and signal.
        state = STATE_PRINT_RESULTS;
        motors.setMotorsPWM(0, 0);
        analogWrite(6, 120);
        delay(5);
        analogWrite(6, 0);
      }
    }

  } else if (state == STATE_PRINT_RESULTS) {

    // Print once per few seconds so the values can be copy-pasted.
    for (int i = 0; i < MAX_RESULTS; i++) {
      Serial.print(results[i]);
      Serial.print(",");
    }
    Serial.print("\n\n\n");
    delay(3000);

  }

}
