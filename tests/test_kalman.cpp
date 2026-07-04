// Host-side tests for the KalmanLine1D line-position estimator.
//
// kalman.h is a self-contained scalar Kalman filter that does not depend on any
// Arduino API, so it compiles and runs directly on the host with g++. These
// tests exercise the same behaviour METHOD_FUSION relies on: fusing the analog
// and digital line positions weighted by each array's live variance (used as
// the measurement noise R).

#include "kalman.h"
#include "test_util.h"

// Two equal measurements with equal variance must fuse to that common value.
// With a diffuse prior (large initial P, i.e. no prior belief about where the
// line is), the estimate is driven entirely by the measurements, so identical
// inputs land on their shared value regardless of the variance level.
static void test_equal_measurements_fuse_to_value() {
  KalmanLine1D k;
  k.reset(0.0, 1.0e6);   // diffuse prior: trust the measurements, not the prior
  float fused = k.fuse(0.3, 0.05, 0.3, 0.05);
  EXPECT_NEAR(fused, 0.3, 1e-3, "equal measurements/variance fuse to that value");

  KalmanLine1D k2;
  k2.reset(0.0, 1.0e6);
  float fused2 = k2.fuse(-0.7, 0.2, -0.7, 0.2);
  EXPECT_NEAR(fused2, -0.7, 1e-3, "equal negative measurements fuse to that value");
}

// A low-variance (trusted) measurement must pull the estimate closer to itself
// than a high-variance (distrusted) one does. Start from a centred estimate,
// apply one measurement at +1.0 with a small R, and check the estimate moves
// most of the way; repeat with a large R and check it barely moves.
static void test_low_variance_pulls_more() {
  KalmanLine1D trusted;
  trusted.reset(0.0, 1.0);
  trusted.update(1.0, 0.001);   // small R: trusted measurement

  KalmanLine1D distrusted;
  distrusted.reset(0.0, 1.0);
  distrusted.update(1.0, 1000.0);   // large R: distrusted measurement

  EXPECT(trusted.x > distrusted.x, "low-variance update pulls estimate further");
  EXPECT(trusted.x > 0.9, "trusted measurement moves estimate most of the way");
  EXPECT(distrusted.x < 0.1, "distrusted measurement barely moves estimate");
}

// Fusing two measurements: the fused estimate must sit closer to the
// lower-variance measurement than to the higher-variance one.
static void test_fusion_biases_toward_trusted() {
  KalmanLine1D k;
  // Analog says -0.5 but is noisy; digital says +0.5 and is trusted.
  float fused = k.fuse(-0.5, 1.0, 0.5, 0.001);
  EXPECT(fused > 0.0, "fused estimate leans toward the trusted (low-var) array");
  EXPECT_NEAR(fused, 0.5, 0.05, "fused estimate is near the trusted measurement");
}

// Feeding a constant true value corrupted by alternating noise must converge
// near the true value as measurements accumulate.
static void test_converges_to_true_value() {
  KalmanLine1D k;
  const float truth = 0.42;
  const float noise[8] = { 0.10, -0.08, 0.05, -0.06, 0.09, -0.07, 0.04, -0.05 };

  for (int n = 0; n < 200; n++) {
    float measurement = truth + noise[n % 8];
    k.predict();
    k.update(measurement, 0.02);
  }

  EXPECT_NEAR(k.x, truth, 0.05, "estimate converges near the constant true value");
}

// R = 0 (a perfectly steady array reporting zero variance) must not divide by
// zero. The eps guard keeps R + eps > 0, so the update stays finite.
static void test_zero_variance_is_safe() {
  KalmanLine1D k;
  k.reset(0.0, 1.0);
  k.update(0.25, 0.0);   // R = 0

  EXPECT(k.x == k.x, "estimate is finite (not NaN) with R = 0");   // NaN != NaN
  EXPECT(k.x > 0.24 && k.x < 0.26, "R=0 gives a near-full-trust update, no blowup");

  KalmanLine1D k3;
  k3.reset(0.0, 1.0e6);   // diffuse prior so the measurements set the value
  float fused = k3.fuse(0.1, 0.0, 0.1, 0.0);
  EXPECT(fused == fused, "fuse with R = 0 stays finite");
  EXPECT_NEAR(fused, 0.1, 1e-3, "fuse with two R=0 measurements lands on the value");
}

int main() {
  printf("Running test_kalman...\n");
  test_equal_measurements_fuse_to_value();
  test_low_variance_pulls_more();
  test_fusion_biases_toward_trusted();
  test_converges_to_true_value();
  test_zero_variance_is_safe();
  return test_summary("test_kalman");
}
