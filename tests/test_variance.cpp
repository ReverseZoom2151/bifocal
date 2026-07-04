// Host-side tests for AnalogLineSensors_c variance logic.
//
// The sensor pipeline is: readAllSensors() averages A_MAX_SAMPLES analogRead
// samples per sensor, getCalibrated() maps that through offset/scale, and
// calculateVariance() takes A_MAX_SAMPLES successive calibrated readings and
// computes the population variance (divide by N, not N-1) per sensor.
//
// We drive analogRead deterministically. One readAllSensors() call performs
// exactly A_MAX_SAMPLES * 5 = 50 analogRead calls, so the "block index"
// index / 50 identifies which readAllSensors (and therefore which getCalibrated
// or which calibrate sample) is in progress. By holding the returned raw value
// constant across a 50-call block we make each averaged reading exact, and by
// stepping the value between blocks we control the sequence of calibrated
// readings that calculateVariance() sees.

#include "arduino_stub.h"
#include "analoglinesensors.h"
#include "test_util.h"

// A_MAX_SAMPLES samples per sensor times 5 sensors per readAllSensors call.
static const unsigned long BLOCK = (unsigned long)A_MAX_SAMPLES * 5UL;

// Calibration sweep: alternate 0 and 1000 between readAllSensors blocks so that
// calibrate() captures min = 0, max = 1000. That yields offset = 0 and
// scale = 1/1000 for every sensor, i.e. calibrated = raw / 1000.
static int calibSweep(int /*pin*/, unsigned long index) {
  unsigned long block = index / BLOCK;
  return (block % 2 == 0) ? 0 : 1000;
}

// Constant raw reading of 500 -> calibrated 0.5 on every sample.
static int constFn(int /*pin*/, unsigned long index) {
  (void)index;
  return 500;
}

// Alternating raw reading 200 / 800 between getCalibrated blocks ->
// calibrated alternates 0.2 / 0.8.
static int alternatingFn(int /*pin*/, unsigned long index) {
  unsigned long block = index / BLOCK;
  return (block % 2 == 0) ? 200 : 800;
}

// Calibrate a fresh sensor object to offset 0, scale 1/1000 using the sweep.
static void calibrateToIdentityScale(AnalogLineSensors_c& s) {
  setAnalogReadFn(calibSweep);
  resetAnalogRead();
  s.calibrate();
}

// Constant input must produce zero variance on every sensor.
static void test_constant_variance_is_zero() {
  AnalogLineSensors_c s;
  calibrateToIdentityScale(s);

  setAnalogReadFn(constFn);
  resetAnalogRead();
  s.calculateVariance();

  for (int i = 0; i < 5; i++) {
    EXPECT_NEAR(s.variance[i], 0.0, 1e-9, "constant input -> variance ~0");
  }

  setAnalogReadFn(constFn);
  resetAnalogRead();
  float avg = s.calculateAverageVariance();
  EXPECT_NEAR(avg, 0.0, 1e-9, "constant input -> average variance ~0");
}

// Alternating input must produce a positive variance that matches the
// population variance formula. With values alternating between a and b in equal
// counts, mean = (a+b)/2 and variance = ((a-b)/2)^2.
static void test_alternating_variance_matches_formula() {
  AnalogLineSensors_c s;
  calibrateToIdentityScale(s);

  // Confirm the calibrated values are exactly what we expect before measuring.
  setAnalogReadFn(alternatingFn);
  resetAnalogRead();
  s.getCalibrated();
  float ca = s.calibrated[0];   // block 0 -> raw 200 -> 0.2
  s.getCalibrated();
  float cb = s.calibrated[0];   // block 1 -> raw 800 -> 0.8
  EXPECT_NEAR(ca, 0.2, 1e-6, "calibrated low value");
  EXPECT_NEAR(cb, 0.8, 1e-6, "calibrated high value");

  // Population variance of an equal alternation of ca and cb.
  double expectedVar = ((ca - cb) / 2.0) * ((ca - cb) / 2.0);

  setAnalogReadFn(alternatingFn);
  resetAnalogRead();
  s.calculateVariance();

  for (int i = 0; i < 5; i++) {
    EXPECT(s.variance[i] > 0.0, "alternating input -> variance > 0");
    EXPECT_NEAR(s.variance[i], expectedVar, 1e-6,
                "alternating variance matches population formula");
  }

  // calculateAverageVariance returns the mean of the 5 per-sensor variances.
  // All sensors are identical here, so the mean equals the per-sensor variance.
  setAnalogReadFn(alternatingFn);
  resetAnalogRead();
  float avg = s.calculateAverageVariance();

  double sum = 0.0;
  for (int i = 0; i < 5; i++) sum += s.variance[i];
  EXPECT_NEAR(avg, sum / 5.0, 1e-9, "average variance equals mean/5");
  EXPECT_NEAR(avg, expectedVar, 1e-6, "average variance matches formula");
}

int main() {
  printf("Running test_variance...\n");
  test_constant_variance_is_zero();
  test_alternating_variance_matches_formula();
  return test_summary("test_variance");
}
