// Host-side tests for the learned sensor-switching policy in policy.h.
//
// policy.h is standalone and AVR-safe: it needs only <math.h> (for logf/expf)
// and the C++ bool type, no Arduino.h. It exposes the coefficients trained
// offline by analysis/train_policy.py plus three inline helpers:
//   policyScore(var_a, var_d, line_err)        signed logistic score
//   policyProbDigital(var_a, var_d, line_err)  probability in [0, 1]
//   preferDigital(var_a, var_d, line_err)      hard decision, true = digital
//
// A positive score / probability > 0.5 means the digital array is the one to
// trust. These tests check the three properties the policy must have:
//   1. clearly-analog-better inputs (var_a much lower) predict analog,
//   2. clearly-digital-better inputs (var_d much lower) predict digital,
//   3. the decision is monotonic in the variance difference: raising var_a with
//      var_d fixed only ever moves the decision toward digital, and raising
//      var_d with var_a fixed only ever moves it toward analog.

#include "policy.h"
#include "test_util.h"

// Clearly-analog-better inputs: the analog array is an order of magnitude or
// more steadier than the digital array. The policy must pick analog.
static void test_clearly_analog_predicts_analog() {
  // var_a well below var_d across a range of absolute magnitudes.
  EXPECT(!preferDigital(1.0e-5f, 1.0e-3f, 0.0f), "tiny var_a -> analog");
  EXPECT(!preferDigital(5.0e-5f, 5.0e-3f, 0.2f), "small var_a -> analog");
  EXPECT(!preferDigital(1.0e-4f, 1.0e-2f, -0.5f), "var_a << var_d -> analog");

  // The probability of preferring digital should be well under 0.5 here.
  EXPECT(policyProbDigital(1.0e-5f, 1.0e-3f, 0.0f) < 0.5f,
         "clearly-analog probability < 0.5");
  EXPECT(policyScore(1.0e-5f, 1.0e-3f, 0.0f) < 0.0f,
         "clearly-analog score < 0");
}

// Clearly-digital-better inputs: the digital array is an order of magnitude or
// more steadier than the analog array (the classic analog-spike case). The
// policy must pick digital.
static void test_clearly_digital_predicts_digital() {
  EXPECT(preferDigital(1.0e-3f, 1.0e-5f, 0.0f), "tiny var_d -> digital");
  EXPECT(preferDigital(5.0e-3f, 5.0e-5f, 0.2f), "small var_d -> digital");
  EXPECT(preferDigital(6.0e-2f, 1.0e-4f, -0.5f), "analog spike -> digital");

  EXPECT(policyProbDigital(1.0e-3f, 1.0e-5f, 0.0f) > 0.5f,
         "clearly-digital probability > 0.5");
  EXPECT(policyScore(1.0e-3f, 1.0e-5f, 0.0f) > 0.0f,
         "clearly-digital score > 0");
}

// The probability form must always lie in [0, 1] and agree with the hard
// decision at the 0.5 threshold.
static void test_probability_bounds_and_agreement() {
  const float vars[] = {1.0e-6f, 1.0e-5f, 1.0e-4f, 1.0e-3f, 1.0e-2f, 6.0e-2f};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      float p = policyProbDigital(vars[i], vars[j], 0.0f);
      EXPECT(p >= 0.0f && p <= 1.0f, "probability within [0,1]");
      bool decision = preferDigital(vars[i], vars[j], 0.0f);
      EXPECT(decision == (p > 0.5f), "hard decision agrees with prob > 0.5");
    }
  }
}

// Monotonic in the variance difference, part 1: with var_d fixed, raising var_a
// must never decrease the score, and the decision may only flip from analog to
// digital (never back). Sweep var_a across four orders of magnitude.
static void test_monotonic_increasing_in_var_a() {
  const float var_d = 1.0e-4f;
  float prev_score = policyScore(1.0e-6f, var_d, 0.0f);
  bool seen_digital = false;

  for (int k = 1; k <= 200; k++) {
    // Geometric sweep from 1e-6 up to 1e-2.
    float t = (float)k / 200.0f;
    float var_a = 1.0e-6f * powf(10.0f, 4.0f * t);

    float score = policyScore(var_a, var_d, 0.0f);
    EXPECT(score >= prev_score - 1.0e-6f, "score nondecreasing as var_a rises");
    prev_score = score;

    bool digital = preferDigital(var_a, var_d, 0.0f);
    if (digital) seen_digital = true;
    // Once we have crossed into "prefer digital", we must not fall back to
    // analog as var_a grows further.
    EXPECT(!(seen_digital && !digital),
           "decision does not flip back to analog as var_a rises");
  }

  // The endpoints must bracket the boundary: analog at the low end, digital at
  // the high end.
  EXPECT(!preferDigital(1.0e-6f, var_d, 0.0f), "low var_a end -> analog");
  EXPECT(preferDigital(1.0e-2f, var_d, 0.0f), "high var_a end -> digital");
}

// Monotonic in the variance difference, part 2: with var_a fixed, raising var_d
// must never increase the score, and the decision may only flip from digital to
// analog (never back).
static void test_monotonic_decreasing_in_var_d() {
  const float var_a = 1.0e-4f;
  float prev_score = policyScore(var_a, 1.0e-6f, 0.0f);
  bool seen_analog = false;

  for (int k = 1; k <= 200; k++) {
    float t = (float)k / 200.0f;
    float var_d = 1.0e-6f * powf(10.0f, 4.0f * t);

    float score = policyScore(var_a, var_d, 0.0f);
    EXPECT(score <= prev_score + 1.0e-6f, "score nonincreasing as var_d rises");
    prev_score = score;

    bool analog = !preferDigital(var_a, var_d, 0.0f);
    if (analog) seen_analog = true;
    EXPECT(!(seen_analog && !analog),
           "decision does not flip back to digital as var_d rises");
  }

  EXPECT(preferDigital(var_a, 1.0e-6f, 0.0f), "low var_d end -> digital");
  EXPECT(!preferDigital(var_a, 1.0e-2f, 0.0f), "high var_d end -> analog");
}

// The line-error feature carries only a small weight, so it must not overturn a
// clear variance-based decision.
static void test_line_error_does_not_dominate() {
  // Clearly analog: extreme line error either way keeps the analog pick.
  EXPECT(!preferDigital(1.0e-5f, 1.0e-3f, 1.0f), "analog holds under +err");
  EXPECT(!preferDigital(1.0e-5f, 1.0e-3f, -1.0f), "analog holds under -err");
  // Clearly digital: same.
  EXPECT(preferDigital(1.0e-3f, 1.0e-5f, 1.0f), "digital holds under +err");
  EXPECT(preferDigital(1.0e-3f, 1.0e-5f, -1.0f), "digital holds under -err");
}

int main() {
  printf("Running test_policy...\n");
  test_clearly_analog_predicts_analog();
  test_clearly_digital_predicts_digital();
  test_probability_bounds_and_agreement();
  test_monotonic_increasing_in_var_a();
  test_monotonic_decreasing_in_var_d();
  test_line_error_does_not_dominate();
  return test_summary("test_policy");
}
