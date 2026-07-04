#ifndef DISAGREEMENT_H
#define DISAGREEMENT_H

// Sensor-disagreement feature detector for the Bifocal line follower.
//
// The robot reads the same line two ways: an analog pipeline
// (AnalogLineSensors_c) and a digital RC-decay pipeline (DigitalLineSensors_c).
// Both expose a calibrated[5] array normalised to roughly 0..1. Most of the
// time the two agree closely. When they disagree sharply that disagreement is
// itself information: it tends to mean a line edge, a junction or branch, or a
// local surface anomaly (glue line, tape seam, reflection). This detector turns
// the analog-vs-digital disagreement into a usable per-tick signal, keeps a
// short rolling baseline of what "normal" disagreement looks like, and raises
// event flags when the current tick spikes above that baseline.
//
// No new hardware. Pure float math, no STL, no heap allocation, AVR-safe.
//
// Intended use (self-contained; not wired into line_following.ino):
//   - Log the disagreement metric alongside the line error for offline tuning.
//   - Trigger recovery or junction-handling logic when isEdgeOrJunction() fires.
//   - Gate the analog/digital switch decision: a strong disagreement tick is a
//     poor moment to trust either single reading, so hold the current choice.
//
// This header may include steering.h for linePosition5(). steering.h references
// the Arduino constrain() macro through PID_c, so on the host it must be
// compiled after an Arduino API shim (arduino_stub.h in tests/).

#include "steering.h"

// Length of the rolling baseline window (number of recent ticks averaged to
// form the "normal" disagreement level). Kept small and fixed so the buffer
// lives on the stack with no allocation.
#ifndef DISAGREE_BASELINE_WINDOW
#define DISAGREE_BASELINE_WINDOW 16
#endif

// Weight of the per-sensor mean-absolute-difference term in the combined tick
// metric. The line-position disagreement term gets the remaining weight
// (1 - this). Range 0..1. Both terms are in comparable 0..1 units.
#ifndef DISAGREE_MAD_WEIGHT
#define DISAGREE_MAD_WEIGHT 0.5
#endif

// Margin, in metric units, that the current tick must exceed the rolling
// baseline by to count as an elevated (candidate edge/junction) event. Needs
// tuning on the robot: without labelled junction data this is only a starting
// guess.
#ifndef DISAGREE_ELEVATED_MARGIN
#define DISAGREE_ELEVATED_MARGIN 0.10
#endif

// Larger margin above the baseline that marks a strong disagreement event
// (very likely a real edge, branch, or anomaly rather than sensor noise).
#ifndef DISAGREE_STRONG_MARGIN
#define DISAGREE_STRONG_MARGIN 0.25
#endif

// Classification of a single tick relative to the rolling baseline.
enum DisagreeLevel {
  DISAGREE_NONE     = 0,   // within normal range
  DISAGREE_ELEVATED = 1,   // above baseline by the elevated margin
  DISAGREE_STRONG   = 2    // above baseline by the strong margin
};

// Free function: mean absolute per-sensor difference between two calibrated[5]
// arrays. Both are already normalised to ~0..1, so this returns a value in the
// same units (0 == identical, larger == more disagreement).
inline float disagreementMAD(float analogCal[5], float digitalCal[5]) {
  float sum = 0.0;
  for (int i = 0; i < 5; i++) {
    float d = analogCal[i] - digitalCal[i];
    if (d < 0.0) d = -d;
    sum += d;
  }
  return sum / 5.0;
}

// Free function: absolute difference between the two line positions, each
// computed with linePosition5() (range [-1, 1]). Result is in 0..2 but in
// practice small; it captures the case where the arrays differ in a way that
// actually moves the estimated line position (the quantity the controller
// steers on) rather than just differing sensor by sensor.
inline float disagreementPosition(float analogCal[5], float digitalCal[5]) {
  float pa = linePosition5(analogCal);
  float pd = linePosition5(digitalCal);
  float d = pa - pd;
  if (d < 0.0) d = -d;
  return d;
}

// Combined single-tick disagreement metric: a weighted blend of the per-sensor
// mean-absolute-difference and the line-position disagreement. Weight is fixed
// at compile time by DISAGREE_MAD_WEIGHT; the class variant below allows a
// runtime weight.
inline float disagreementMetric(float analogCal[5], float digitalCal[5]) {
  float w = DISAGREE_MAD_WEIGHT;
  return w * disagreementMAD(analogCal, digitalCal)
       + (1.0 - w) * disagreementPosition(analogCal, digitalCal);
}

// Rolling detector: keeps a short ring buffer of recent tick metrics to form a
// baseline, then flags ticks that spike above it. Stateful; feed it once per
// control loop via update().
class DisagreementDetector_c {

  private:

    // Rolling baseline ring buffer of recent tick metrics and its running sum
    // for an O(1) mean. Seeded on the first update() so early ticks compare
    // against a sane baseline instead of zero.
    float ring[DISAGREE_BASELINE_WINDOW];
    float ring_sum;
    int   ring_head;
    bool  seeded;

    // Configurable weight and thresholds. Defaults come from the #defines and
    // can be overridden at runtime by the setters.
    float mad_weight;
    float elevated_margin;
    float strong_margin;

    // State from the most recent update().
    float last_metric;
    float last_baseline;
    int   last_level;

  public:

    DisagreementDetector_c() {
      mad_weight      = DISAGREE_MAD_WEIGHT;
      elevated_margin = DISAGREE_ELEVATED_MARGIN;
      strong_margin   = DISAGREE_STRONG_MARGIN;
      reset();
    }

    // Clear the rolling baseline and last-tick state. Does not change the
    // configured weight or thresholds.
    void reset() {
      ring_sum      = 0.0;
      ring_head     = 0;
      seeded        = false;
      last_metric   = 0.0;
      last_baseline = 0.0;
      last_level    = DISAGREE_NONE;
      for (int i = 0; i < DISAGREE_BASELINE_WINDOW; i++) ring[i] = 0.0;
    }

    // Set the blend weight of the per-sensor MAD term (0..1). Out-of-range
    // values are ignored.
    void setMadWeight(float w) {
      if (w >= 0.0 && w <= 1.0) mad_weight = w;
    }

    // Set the elevated-event margin above baseline (> 0). Ignored otherwise.
    void setElevatedMargin(float m) {
      if (m > 0.0) elevated_margin = m;
    }

    // Set the strong-event margin above baseline (> 0). Ignored otherwise.
    void setStrongMargin(float m) {
      if (m > 0.0) strong_margin = m;
    }

    // Combined tick metric using the runtime weight (not the compile-time one).
    float metricFor(float analogCal[5], float digitalCal[5]) {
      return mad_weight * disagreementMAD(analogCal, digitalCal)
           + (1.0 - mad_weight) * disagreementPosition(analogCal, digitalCal);
    }

    // Feed one control tick. Computes the combined disagreement metric for the
    // two calibrated arrays, classifies it against the current rolling baseline
    // (the baseline excludes this tick, so a lone spike is caught relative to
    // recent normal), then folds this tick into the baseline. Returns the
    // metric. The spike, once sustained, raises the baseline and the flag
    // clears: the detector adapts to a persistently noisy surface.
    float update(float analogCal[5], float digitalCal[5]) {

      float metric = metricFor(analogCal, digitalCal);

      // Seed the whole window on the first tick so the baseline starts at the
      // observed level rather than zero (which would false-flag tick one).
      if (!seeded) {
        ring_sum = 0.0;
        for (int i = 0; i < DISAGREE_BASELINE_WINDOW; i++) {
          ring[i]   = metric;
          ring_sum += metric;
        }
        ring_head     = 0;
        seeded        = true;
        last_metric   = metric;
        last_baseline = metric;
        last_level    = DISAGREE_NONE;
        return metric;
      }

      float baseline = ring_sum / (float)DISAGREE_BASELINE_WINDOW;
      float excess   = metric - baseline;

      int level = DISAGREE_NONE;
      if (excess >= strong_margin)        level = DISAGREE_STRONG;
      else if (excess >= elevated_margin) level = DISAGREE_ELEVATED;

      last_metric   = metric;
      last_baseline = baseline;
      last_level    = level;

      // Fold this tick into the rolling baseline (O(1)).
      float old = ring[ring_head];
      ring_sum -= old;
      ring[ring_head] = metric;
      ring_sum += metric;
      ring_head = (ring_head + 1) % DISAGREE_BASELINE_WINDOW;

      return metric;
    }

    // Metric of the most recent update().
    float lastMetric() const { return last_metric; }

    // Rolling baseline the most recent tick was compared against.
    float baseline() const { return last_baseline; }

    // How far the last tick sat above the baseline (may be negative).
    float lastExcess() const { return last_metric - last_baseline; }

    // Classification of the most recent tick.
    int level() const { return last_level; }

    // True when the most recent tick is at least an elevated event: a candidate
    // line edge, junction/branch, or surface anomaly worth reacting to.
    bool isEdgeOrJunction() const { return last_level != DISAGREE_NONE; }

    // True only for a strong event (high confidence it is a real feature).
    bool isStrong() const { return last_level == DISAGREE_STRONG; }

};

#endif
