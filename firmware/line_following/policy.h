#ifndef POLICY_H
#define POLICY_H

/*
 * Learned sensor-switching policy for the line-following robot.
 *
 * These coefficients are produced OFFLINE by analysis/train_policy.py and
 * written in automatically by that script. Do not edit them by hand; re-run
 *   python analysis/train_policy.py
 * to regenerate this header from the training data.
 *
 * The policy is a logistic regression over four cheap per-tick features:
 *   var_a      analog average variance
 *   var_d      digital average variance
 *   log_ratio  log((var_a+eps)/(var_d+eps)), the scale-invariant compare
 *   abs_err    magnitude of the recent line-position error, |line_error|
 *
 * score = BIAS + W_VARA*var_a + W_VARD*var_d
 *              + W_LRATIO*log_ratio + W_ERR*abs_err
 * probability of preferring the digital array = sigmoid(score).
 * preferDigital() returns true when that probability exceeds 0.5.
 *
 * The offline fit standardized the features; that standardization is folded
 * into the constants below, so the device applies RAW features directly with
 * no per-feature mean or scale needed on-board. Cost per call: one divide,
 * one logf, a few multiply-adds (plus one expf only if the probability form
 * is used; preferDigital() needs just the sign of the score).
 *
 * Training data is SYNTHETIC (see train_policy.py) until real labeled runs
 * exist. Coefficients are reproducible with np.random.default_rng(0).
 *
 * Fit quality on the synthetic set:
 *   train accuracy = 0.8313  (n = 40000)
 *   test  accuracy = 0.8302  (n = 20000)
 */

#include <math.h>   // logf, expf

// Guard added inside the ratio so a near-zero variance cannot divide by
// zero or hit log(0). MUST match VAR_EPS in analysis/train_policy.py.
static const float POLICY_VAR_EPS = 1e-07f;

// Raw-feature logistic-regression coefficients (standardization folded in).
static const float POLICY_BIAS     = 0.11242007f;
static const float POLICY_W_VARA   = -9.4613028f;
static const float POLICY_W_VARD   = 27.446514f;
static const float POLICY_W_LRATIO = 0.92805463f;
static const float POLICY_W_ERR    = 0.015219776f;

// Flat coefficient array {bias, var_a, var_d, log_ratio, abs_err}, handy for
// logging or bulk copies. Kept in sync with the named constants above.
static const float POLICY_COEFFS[5] = {
  0.11242007f, -9.4613028f, 27.446514f, 0.92805463f, 0.015219776f
};

// Linear logistic score. A positive score favors the digital array.
// var_a, var_d are the two arrays' average variances this tick; line_err is
// the recent line-position error in [-1, 1].
static inline float policyScore(float var_a, float var_d, float line_err) {
  float log_ratio = logf((var_a + POLICY_VAR_EPS) / (var_d + POLICY_VAR_EPS));
  float abs_err   = (line_err < 0.0f) ? -line_err : line_err;
  return POLICY_BIAS
       + POLICY_W_VARA   * var_a
       + POLICY_W_VARD   * var_d
       + POLICY_W_LRATIO * log_ratio
       + POLICY_W_ERR    * abs_err;
}

// Probability in [0, 1] that the digital array is the one to trust.
static inline float policyProbDigital(float var_a, float var_d, float line_err) {
  return 1.0f / (1.0f + expf(-policyScore(var_a, var_d, line_err)));
}

// Hard decision: true = prefer the digital array, false = prefer analog.
// Equivalent to policyProbDigital(...) > 0.5 but skips the sigmoid.
static inline bool preferDigital(float var_a, float var_d, float line_err) {
  return policyScore(var_a, var_d, line_err) > 0.0f;
}

#endif  // POLICY_H
