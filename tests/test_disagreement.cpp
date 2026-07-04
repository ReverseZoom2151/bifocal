// Host-side unit tests for disagreement.h.
//
// disagreement.h includes steering.h, which references the Arduino constrain()
// macro through PID_c, so arduino_stub.h must be included first to supply the
// Arduino API shim. No hardware and no external framework required.

#include "arduino_stub.h"
#include "disagreement.h"
#include "test_util.h"

// Helper to fill a calibrated[5] array.
static void setArr(float a[5], float v0, float v1, float v2, float v3, float v4) {
  a[0] = v0; a[1] = v1; a[2] = v2; a[3] = v3; a[4] = v4;
}

// Identical analog and digital arrays: metric ~0, no event.
static void test_identical_no_event() {
  float analog[5];
  float digital[5];
  setArr(analog,  0.1, 0.2, 0.9, 0.2, 0.1);
  setArr(digital, 0.1, 0.2, 0.9, 0.2, 0.1);

  EXPECT_NEAR(disagreementMAD(analog, digital), 0.0, 1e-6,
              "identical arrays give zero MAD");
  EXPECT_NEAR(disagreementPosition(analog, digital), 0.0, 1e-6,
              "identical arrays give zero position disagreement");

  DisagreementDetector_c det;
  for (int t = 0; t < 20; t++) {
    det.update(analog, digital);
    EXPECT(!det.isEdgeOrJunction(), "no event when arrays always agree");
  }
  EXPECT_NEAR(det.lastMetric(), 0.0, 1e-6, "metric stays ~0 when identical");
  EXPECT(det.level() == DISAGREE_NONE, "level NONE when identical");
}

// A large induced difference raises the disagreement and sets the event flag
// above baseline.
static void test_spike_sets_event() {
  float analog[5];
  float digital[5];
  setArr(analog,  0.0, 0.0, 1.0, 0.0, 0.0);   // line centred, analog view
  setArr(digital, 0.0, 0.0, 1.0, 0.0, 0.0);

  DisagreementDetector_c det;

  // Establish a quiet baseline: many ticks of perfect agreement.
  for (int t = 0; t < 20; t++) det.update(analog, digital);
  EXPECT(!det.isEdgeOrJunction(), "quiet baseline: no event");
  float quietMetric = det.lastMetric();

  // One tick where the two pipelines strongly disagree (digital sees the line
  // hard to the right, analog still centred: like a branch entering).
  setArr(digital, 0.0, 0.0, 0.0, 0.0, 1.0);
  det.update(analog, digital);

  EXPECT(det.lastMetric() > quietMetric + 0.1, "spike raises the metric");
  EXPECT(det.isEdgeOrJunction(), "spike above baseline sets the event flag");
  EXPECT(det.isStrong(), "large spike classifies as strong");
  EXPECT(det.lastExcess() > 0.0, "spike sits above the baseline");
}

// The rolling baseline adapts: a persistently high disagreement stops firing
// the event once the baseline has caught up to it.
static void test_baseline_adapts() {
  float analog[5];
  float digital[5];
  setArr(analog,  0.0, 0.0, 1.0, 0.0, 0.0);
  setArr(digital, 0.0, 0.0, 1.0, 0.0, 0.0);

  DisagreementDetector_c det;

  // Quiet baseline first.
  for (int t = 0; t < 20; t++) det.update(analog, digital);

  // Switch to a sustained disagreement. The first such tick should fire.
  setArr(digital, 0.0, 0.5, 1.0, 0.5, 0.0);
  det.update(analog, digital);
  EXPECT(det.isEdgeOrJunction(), "first sustained-disagreement tick fires");

  // Hold the same disagreement for a full window plus margin. The rolling mean
  // climbs toward the new level, so the excess over baseline shrinks and the
  // event clears: the detector has adapted to the noisier surface.
  for (int t = 0; t < DISAGREE_BASELINE_WINDOW + 4; t++) {
    det.update(analog, digital);
  }
  EXPECT(!det.isEdgeOrJunction(),
         "sustained disagreement no longer fires after baseline adapts");
  EXPECT(det.baseline() > 0.05, "baseline rose toward the sustained level");

  // A fresh, even larger jump on top of the adapted baseline fires again.
  setArr(digital, 1.0, 1.0, 0.0, 1.0, 1.0);
  det.update(analog, digital);
  EXPECT(det.isEdgeOrJunction(), "new larger spike above adapted baseline fires");
}

// Configurable thresholds via setters take effect.
static void test_configurable_thresholds() {
  float analog[5];
  float digital[5];
  setArr(analog,  0.0, 0.0, 1.0, 0.0, 0.0);
  setArr(digital, 0.0, 0.0, 1.0, 0.0, 0.0);

  DisagreementDetector_c det;
  // Make the detector very insensitive: huge margins.
  det.setElevatedMargin(5.0);
  det.setStrongMargin(9.0);

  for (int t = 0; t < 20; t++) det.update(analog, digital);
  setArr(digital, 0.0, 0.0, 0.0, 0.0, 1.0);
  det.update(analog, digital);
  EXPECT(!det.isEdgeOrJunction(),
         "with huge margins even a real spike does not fire");

  // Now a very sensitive detector fires on the same spike.
  DisagreementDetector_c det2;
  det2.setElevatedMargin(0.01);
  det2.setStrongMargin(0.02);
  setArr(digital, 0.0, 0.0, 1.0, 0.0, 0.0);
  for (int t = 0; t < 20; t++) det2.update(analog, digital);
  setArr(digital, 0.0, 0.0, 0.0, 0.0, 1.0);
  det2.update(analog, digital);
  EXPECT(det2.isEdgeOrJunction(), "with tiny margins the spike fires");
}

int main() {
  printf("=== test_disagreement ===\n");
  test_identical_no_event();
  test_spike_sets_event();
  test_baseline_adapts();
  test_configurable_thresholds();
  return test_summary("test_disagreement");
}
