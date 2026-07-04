// odometry_calibration.ino
//
// Standalone helper sketch for calibrating the three physical constants used by
// Kinematics_c (see firmware/line_following/kinematics.h):
//
//   wheelRadius                 (mm)
//   wheelSeparation             (mm)
//   encoderCountsPerRevolution  (counts per wheel revolution)
//
// It does not follow a line. It runs two guided tests and prints the raw
// encoder counts and the derived ratios over serial so a user can compute
// calibrated values by hand and then plug them into
// kinematics.setGeometry(...). The full written procedure and formulas live in
// docs/odometry-calibration.md.
//
// TEST 1 (straight line): drive forward a known distance, then read the counts.
//         This gives mmPerCount and, with the nominal wheel radius, an estimate
//         of encoderCountsPerRevolution.
// TEST 2 (rotation):      spin in place through a full 360 degree turn, then
//         read the counts. Combined with the wheel radius this gives the
//         effective wheelSeparation.
//
// INCLUDE PATHS
// This sketch reuses encoders.h and motors.h from the main firmware, which live
// one level up at ../../line_following/. The relative includes below work when
// the tree layout is preserved. If your Arduino setup rejects the relative
// paths (some IDE versions restrict includes to the sketch folder), copy
// encoders.h and motors.h into this folder next to this .ino file and change
// the two includes to "encoders.h" and "motors.h".

#include "../../line_following/encoders.h"
#include "../../line_following/motors.h"

// -------------------------------------------------------------------------
// Test parameters. Adjust these to suit your bench and floor surface.
// -------------------------------------------------------------------------

// Nominal geometry, only used to turn measured counts into first-guess
// physical numbers in the printed output. The measured counts themselves are
// the primary result and do not depend on these.
const float NOMINAL_WHEEL_RADIUS_MM = 16.5;

// The straight-line test distance. Mark this distance on the floor with a ruler
// or tape. A longer distance averages out per-count error, so prefer 500 mm or
// more if you have the space.
const float STRAIGHT_DISTANCE_MM = 500.0;

// PWM magnitudes for the two tests. Keep them low so the wheels do not slip,
// which would corrupt the counts. Increase only if the robot stalls.
const int STRAIGHT_PWM = 40;
const int SPIN_PWM     = 40;

// How long each motion runs, in milliseconds. These are open loop times, not
// precise, so you will nudge them by trial and error until the robot travels
// close to STRAIGHT_DISTANCE_MM or completes exactly one 360 degree turn.
const unsigned long STRAIGHT_TIME_MS = 2500;
const unsigned long SPIN_TIME_MS     = 2500;

// Seconds to wait between the on-screen instruction and the motion starting, so
// the user can line the robot up.
const int COUNTDOWN_S = 5;

Motors_c motors;

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

// Print a simple countdown so the user can position the robot before motion.
void countdown(int seconds) {
  for (int i = seconds; i > 0; i--) {
    Serial.print("  starting in ");
    Serial.print(i);
    Serial.println(" ...");
    delay(1000);
  }
}

// Run the straight-line test. Drives both wheels forward for a fixed time, then
// reports the counts and the ratios derived from them.
void runStraightTest() {
  Serial.println();
  Serial.println("TEST 1: STRAIGHT LINE");
  Serial.println("  Place the robot on a flat surface with clear space ahead.");
  Serial.print("  It will drive forward for about ");
  Serial.print(STRAIGHT_DISTANCE_MM);
  Serial.println(" mm.");
  Serial.println("  Measure the ACTUAL distance travelled with a ruler after it stops.");
  countdown(COUNTDOWN_S);

  // Zero both counters at the start of the motion.
  count_e0 = 0;
  count_e1 = 0;

  motors.driveStraight(STRAIGHT_PWM);
  delay(STRAIGHT_TIME_MS);
  motors.setMotorsPWM(0, 0);

  // Snapshot the volatile counters into locals for printing.
  long left  = count_e0;
  long right = count_e1;
  long average = (left + right) / 2;

  Serial.println("  --- straight test result ---");
  Serial.print("  left count  (count_e0) = ");
  Serial.println(left);
  Serial.print("  right count (count_e1) = ");
  Serial.println(right);
  Serial.print("  average count          = ");
  Serial.println(average);

  if (average != 0) {
    // mmPerCount using the COMMANDED distance. Replace STRAIGHT_DISTANCE_MM in
    // your notes with the distance you actually measured for the real value.
    float mmPerCount = STRAIGHT_DISTANCE_MM / (float)average;
    Serial.print("  mmPerCount (using commanded distance) = ");
    Serial.println(mmPerCount, 5);

    // Counts per revolution implied by the nominal wheel radius.
    float wheelCircumference = 2.0 * PI * NOMINAL_WHEEL_RADIUS_MM;
    float countsPerRev = wheelCircumference / mmPerCount;
    Serial.print("  implied countsPerRev (nominal radius) = ");
    Serial.println(countsPerRev, 3);
  }
  Serial.println("  Recompute mmPerCount with your MEASURED distance:");
  Serial.println("    mmPerCount = measuredDistanceMm / averageCount");
}

// Run the rotation test. Spins in place for a fixed time, then reports the
// counts and the derived wheel separation.
void runSpinTest() {
  Serial.println();
  Serial.println("TEST 2: ROTATION (360 DEGREES)");
  Serial.println("  Mark the robot's heading (a piece of tape on the floor helps).");
  Serial.println("  It will spin in place. Aim for exactly one full turn.");
  Serial.println("  Adjust SPIN_TIME_MS in this sketch until it lands on 360 degrees.");
  countdown(COUNTDOWN_S);

  count_e0 = 0;
  count_e1 = 0;

  // Spin in place: one wheel forward, one wheel back. spinLeft handles the
  // direction pins; the delay argument holds the motion for the given time.
  motors.spinLeft(SPIN_PWM, SPIN_TIME_MS);
  motors.setMotorsPWM(0, 0);

  long left  = count_e0;
  long right = count_e1;

  // For a pure spin the two wheels move in opposite directions, so take
  // magnitudes and average them to get the per-wheel arc count.
  long absLeft  = left  < 0 ? -left  : left;
  long absRight = right < 0 ? -right : right;
  long arcCount = (absLeft + absRight) / 2;

  Serial.println("  --- rotation test result ---");
  Serial.print("  left count  (count_e0) = ");
  Serial.println(left);
  Serial.print("  right count (count_e1) = ");
  Serial.println(right);
  Serial.print("  |avg| arc count        = ");
  Serial.println(arcCount);
  Serial.println("  If this was NOT a clean 360 turn, adjust SPIN_TIME_MS and repeat.");
  Serial.println("  For a full 360 turn each wheel rolls half the turning circle:");
  Serial.println("    wheelSeparation = (2 * arcCount * mmPerCount) / PI");
  Serial.println("  Use the mmPerCount you found in TEST 1.");
}

// -------------------------------------------------------------------------
// Arduino entry points
// -------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  delay(1000);

  setupEncoder0();
  setupEncoder1();
  motors.initialise();

  Serial.println("***ODOMETRY CALIBRATION HELPER***");
  Serial.println("Two tests run once each, then the sketch idles.");
  Serial.println("Read docs/odometry-calibration.md for the full procedure.");

  runStraightTest();
  runSpinTest();

  Serial.println();
  Serial.println("Done. Press reset to run the tests again.");
  Serial.println("Plug your numbers into kinematics.setGeometry(radius, separation, countsPerRev).");
}

void loop() {
  // Nothing to do. The tests run once from setup().
}
