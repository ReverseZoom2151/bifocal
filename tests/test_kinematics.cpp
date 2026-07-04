// Host-side tests for Kinematics_c dead-reckoning odometry.
//
// update(left, right) integrates one differential-drive step from absolute
// encoder counts:
//   dL = left  - lastLeft,  dR = right - lastRight
//   distL = dL * mmPerCount, distR = dR * mmPerCount
//   xPos += avg(distL, distR) * cos(theta)
//   yPos += avg(distL, distR) * sin(theta)
//   theta += (distR - distL) / wheelSeparation
// The first call measures deltas from the initial counts of 0.

#include "arduino_stub.h"
#include "kinematics.h"
#include "test_util.h"

// Driving both wheels forward by equal counts advances xPos and leaves theta
// at ~0 (heading starts at 0, so motion is along +x).
static void test_straight_drive() {
  Kinematics_c k;
  double mmPerCount = k.getMmPerCount();

  long counts = 100;
  k.update(counts, counts);

  double expectedX = (double)counts * mmPerCount;
  EXPECT_NEAR(k.xPos, expectedX, 1e-4, "straight drive advances xPos");
  EXPECT_NEAR(k.yPos, 0.0, 1e-9, "straight drive keeps yPos ~0");
  EXPECT_NEAR(k.theta, 0.0, 1e-12, "straight drive keeps theta ~0");
}

// Equal and opposite wheel counts rotate in place: no net translation, and
// theta changes by (distR - distL) / wheelSeparation.
static void test_rotation_in_place() {
  Kinematics_c k;
  double mmPerCount = k.getMmPerCount();
  double sep = k.getWheelSeparation();

  long n = 50;                 // left goes back n, right goes forward n
  k.update(-n, n);

  double distL = (double)(-n) * mmPerCount;
  double distR = (double)(n) * mmPerCount;
  double expectedTheta = (distR - distL) / sep;

  // theta is a float, so allow for single-precision rounding.
  EXPECT_NEAR(k.theta, expectedTheta, 1e-5,
              "opposite counts rotate theta by (dR-dL)/wheelSeparation");
  EXPECT_NEAR(k.xPos, 0.0, 1e-5, "pure rotation leaves xPos ~0");
  EXPECT_NEAR(k.yPos, 0.0, 1e-5, "pure rotation leaves yPos ~0");
  EXPECT(k.theta > 0.0, "positive (dR-dL) yields positive rotation");
}

// Zero counts must not change the pose.
static void test_zero_counts_no_change() {
  Kinematics_c k;
  k.update(0, 0);
  EXPECT_NEAR(k.xPos, 0.0, 1e-12, "zero counts leave xPos unchanged");
  EXPECT_NEAR(k.yPos, 0.0, 1e-12, "zero counts leave yPos unchanged");
  EXPECT_NEAR(k.theta, 0.0, 1e-12, "zero counts leave theta unchanged");

  // A repeated identical count also produces no delta and no change.
  Kinematics_c k2;
  k2.update(200, 200);
  double xAfterFirst = k2.xPos;
  k2.update(200, 200);
  EXPECT_NEAR(k2.xPos, xAfterFirst, 1e-12, "repeated same count adds nothing");
  EXPECT_NEAR(k2.theta, 0.0, 1e-12, "repeated same count keeps theta ~0");
}

int main() {
  printf("Running test_kinematics...\n");
  test_straight_drive();
  test_rotation_in_place();
  test_zero_counts_no_change();
  return test_summary("test_kinematics");
}
