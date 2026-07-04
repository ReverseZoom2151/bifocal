// Host-side tests for AnalogLineSensors_c variance logic.
//
// The sensor pipeline is: readAllSensors() averages A_MAX_SAMPLES analogRead
// samples per sensor, getCalibrated() maps that through offset/scale and an
// optional per-sensor gain trim, applies an outlier clamp against the window
// median, then pushes the value into a per-sensor ring buffer of the last
// A_MAX_SAMPLES calibrated readings. variance[] holds the population variance
// (divide by N, not N-1) over that rolling window and is updated on every
// getCalibrated() call, so it is O(1) and non-blocking. calculateVariance()
// takes one fresh reading and refreshes the rolling estimate;
// calculateAverageVariance() does the same and returns the mean of variance[].
//
// We drive analogRead deterministically. One readAllSensors() call performs
// exactly A_MAX_SAMPLES * 5 = 50 analogRead calls, so the "block index"
// index / 50 identifies which readAllSensors (and therefore which getCalibrated
// or which calibrate sample) is in progress. Holding the returned raw value
// constant across a 50-call block makes each averaged reading exact; stepping
// the value between blocks controls the sequence of calibrated readings.

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

// Constant raw reading of 900 -> calibrated 0.9 (used to inject one spike).
static int spikeFn(int /*pin*/, unsigned long index) {
  (void)index;
  return 900;
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

// Constant input must produce zero variance on every sensor once the rolling
// window has filled with that constant.
static void test_constant_variance_is_zero() {
  AnalogLineSensors_c s;
  calibrateToIdentityScale(s);

  setAnalogReadFn(constFn);
  resetAnalogRead();
  for (int n = 0; n < 3 * A_MAX_SAMPLES; n++) s.getCalibrated();

  for (int i = 0; i < 5; i++) {
    EXPECT_NEAR(s.variance[i], 0.0, 1e-9, "constant input -> variance ~0");
  }

  float avg = s.calculateAverageVariance();
  EXPECT_NEAR(avg, 0.0, 1e-9, "constant input -> average variance ~0");
}

// Alternating input must produce a positive variance that matches the
// population variance formula. With values alternating between a and b in equal
// counts (the window size A_MAX_SAMPLES is even), any A_MAX_SAMPLES consecutive
// samples hold an even split, so variance = ((a-b)/2)^2. We disable the outlier
// clamp here so it does not squash the alternation, isolating the variance math.
static void test_alternating_variance_matches_formula() {
  AnalogLineSensors_c s;
  calibrateToIdentityScale(s);
  s.setOutlierClamp(1.0e9);   // disable spike clamping for the raw-formula test

  setAnalogReadFn(alternatingFn);
  resetAnalogRead();

  // Fill the rolling window well past its size so the seed sample is gone and
  // the window holds a clean 0.2 / 0.8 alternation.
  for (int n = 0; n < 4 * A_MAX_SAMPLES; n++) s.getCalibrated();

  // Confirm the calibrated values are the unclamped 0.2 / 0.8 we expect.
  s.getCalibrated();
  float c1 = s.calibrated[0];
  s.getCalibrated();
  float c2 = s.calibrated[0];
  float lo = (c1 < c2) ? c1 : c2;
  float hi = (c1 < c2) ? c2 : c1;
  EXPECT_NEAR(lo, 0.2, 1e-6, "calibrated low value");
  EXPECT_NEAR(hi, 0.8, 1e-6, "calibrated high value");

  // Population variance of an equal alternation of 0.2 and 0.8.
  double expectedVar = ((0.2 - 0.8) / 2.0) * ((0.2 - 0.8) / 2.0);   // 0.09

  for (int i = 0; i < 5; i++) {
    EXPECT(s.variance[i] > 0.0, "alternating input -> variance > 0");
    EXPECT_NEAR(s.variance[i], expectedVar, 1e-6,
                "alternating variance matches population formula");
  }

  // calculateAverageVariance returns the mean of the 5 per-sensor variances.
  // All sensors are identical here, so the mean equals the per-sensor variance.
  float avg = s.calculateAverageVariance();
  double sum = 0.0;
  for (int i = 0; i < 5; i++) sum += s.variance[i];
  EXPECT_NEAR(avg, sum / 5.0, 1e-9, "average variance equals mean/5");
  EXPECT_NEAR(avg, expectedVar, 1e-6, "average variance matches formula");
}

// The outlier clamp must reject a single large spike: after filling the window
// with a steady value, a lone reading far outside the clamp band is pulled back
// to within outlier_clamp of the window median instead of passing through.
static void test_outlier_clamp_rejects_spike() {
  AnalogLineSensors_c s;
  calibrateToIdentityScale(s);
  s.setOutlierClamp(0.3);   // explicit, matches the default

  // Fill the window with a steady 0.5 so the median is 0.5.
  setAnalogReadFn(constFn);
  resetAnalogRead();
  for (int n = 0; n < 3 * A_MAX_SAMPLES; n++) s.getCalibrated();

  // Inject one 0.9 reading. Median is 0.5, so it must clamp to 0.5 + 0.3 = 0.8.
  setAnalogReadFn(spikeFn);
  s.getCalibrated();
  EXPECT_NEAR(s.calibrated[0], 0.8, 1e-6, "spike clamped to median + clamp");
}

int main() {
  printf("Running test_variance...\n");
  test_constant_variance_is_zero();
  test_alternating_variance_matches_formula();
  test_outlier_clamp_rejects_spike();
  return test_summary("test_variance");
}
