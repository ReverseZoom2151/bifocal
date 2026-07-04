#!/usr/bin/env python3
"""Train the learned sensor-switching policy for the line-following robot.

Run from the repo root:  python analysis/train_policy.py

Today the firmware picks between the analog and the digital IR array by
comparing their average reading variances against a hand-set margin
(SWITCH_MARGIN in line_following.ino). This script replaces that guesswork with
a tiny logistic-regression policy: given a few cheap per-tick features it
predicts which array to trust. The trained model is a handful of float
coefficients that the device applies with a short expression (a couple of
multiply-adds plus one logf), so it costs almost nothing on the AVR.

Pipeline:
  1. Synthesize a labeled training set (self-contained, no sim binary). For each
     sample we draw a "true" noise level for each array from plausible
     log-normal distributions matched to data/results (analog medians around
     1e-4 with occasional large spikes, digital tighter around 5e-5). The
     variance the controller actually observes is that true level times a
     measurement-noise factor (finite rolling window). The correct label is
     which array had the lower TRUE noise. A fraction of labels is flipped to
     model real-world label noise.
  2. Fit a logistic regression with numpy (hand-rolled gradient descent, so
     there is no hard dependency on sklearn; sklearn is used only if present).
     Features are standardized during the fit for conditioning.
  3. Fold the standardization back into the exported weights so the device
     applies RAW features directly (no per-feature mean/scale needed on-board).
  4. Report train/test accuracy and write the coefficients into
     firmware/line_following/policy.h.

Randomness is fully seeded (np.random.default_rng(0)); there is no time-based
randomness, so re-running reproduces the same coefficients.

Features (all available per control tick in line_following.ino):
  f0 = var_a                          analog average variance
  f1 = var_d                          digital average variance
  f2 = log((var_a+eps)/(var_d+eps))   log variance ratio (scale-invariant)
  f3 = |line_error|                   magnitude of the recent line-position error

The log variance ratio is the dominant signal. A raw var_a - var_d difference
was tried first but caps out well below the log ratio, because comparing two
variances that span several orders of magnitude is a ratio question, not a
difference question. var_a and var_d are kept as raw features so isolated
spikes still carry weight.

Label convention: 1 = prefer the digital array, 0 = prefer the analog array.
"""
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POLICY_H = os.path.join(ROOT, "firmware", "line_following", "policy.h")

# Small guard added inside the ratio/log so a near-zero variance cannot divide
# by zero or take log(0). MUST match POLICY_VAR_EPS in policy.h.
VAR_EPS = 1.0e-7

# Feature names, in the fixed order used everywhere below and in policy.h.
FEATURES = ["var_a", "var_d", "log_ratio", "abs_err"]


def make_features(var_a, var_d, line_err):
    """Assemble the raw feature matrix from the three per-tick quantities."""
    log_ratio = np.log((var_a + VAR_EPS) / (var_d + VAR_EPS))
    return np.column_stack([var_a, var_d, log_ratio, np.abs(line_err)])


def synthesize(rng, n):
    """Return (X, y) with X shape (n, 4) of raw features and y in {0,1}.

    y = 1 means the digital array truly had the lower noise for that sample.
    """
    # True noise levels, drawn in log10 space to span the orders of magnitude
    # seen in data/results. Analog is centered a touch higher and has a heavier
    # tail; digital is tighter and spikes rarely.
    log_true_a = rng.normal(-4.0, 0.55, size=n)   # median ~1e-4
    log_true_d = rng.normal(-4.25, 0.40, size=n)  # median ~5.6e-5, tighter

    # Occasional large analog spikes (dust, glare, line straddle), matching the
    # long analog tail (observed max around 6e-2). Add a big bump to a subset.
    spike_a = rng.random(n) < 0.15
    log_true_a[spike_a] += rng.uniform(1.0, 2.5, size=spike_a.sum())

    # Digital spikes much more rarely and by less.
    spike_d = rng.random(n) < 0.04
    log_true_d[spike_d] += rng.uniform(0.5, 1.2, size=spike_d.sum())

    true_a = 10.0 ** log_true_a
    true_d = 10.0 ** log_true_d

    # The controller does not see the true noise; it sees a variance estimated
    # over a short rolling window, so multiply by a log-normal measurement
    # factor (unbiased in log space).
    meas_a = 10.0 ** rng.normal(0.0, 0.22, size=n)
    meas_d = 10.0 ** rng.normal(0.0, 0.22, size=n)
    var_a = true_a * meas_a
    var_d = true_d * meas_d

    # Recent line-position error in [-1, 1]. It is only weakly informative, so
    # the fit should give it a small weight rather than leaning on it.
    line_err = rng.uniform(-1.0, 1.0, size=n)

    # Correct label from the TRUE noise levels.
    y = (true_d < true_a).astype(np.float64)

    # Flip a fraction of labels to model real-world label noise.
    flip = rng.random(n) < 0.08
    y[flip] = 1.0 - y[flip]

    X = make_features(var_a, var_d, line_err)
    return X, y


def sigmoid(z):
    # Numerically stable logistic.
    out = np.empty_like(z)
    pos = z >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-z[pos]))
    ez = np.exp(z[~pos])
    out[~pos] = ez / (1.0 + ez)
    return out


def fit_logreg(Xs, y, epochs=4000, lr=0.5, l2=1e-4):
    """Hand-rolled logistic regression via batch gradient descent on
    standardized features Xs. Returns (weights, bias)."""
    n, d = Xs.shape
    w = np.zeros(d)
    b = 0.0
    for _ in range(epochs):
        p = sigmoid(Xs @ w + b)
        err = p - y
        grad_w = Xs.T @ err / n + l2 * w
        grad_b = err.mean()
        w -= lr * grad_w
        b -= lr * grad_b
    return w, b


def try_sklearn(Xs, y):
    """Optional: prefer sklearn if it happens to be installed, otherwise the
    caller falls back to the hand-rolled fit. Kept optional so this script has
    no hard dependency beyond numpy."""
    try:
        from sklearn.linear_model import LogisticRegression
    except Exception:
        return None
    clf = LogisticRegression(C=1.0, max_iter=5000)
    clf.fit(Xs, y)
    return clf.coef_[0].copy(), float(clf.intercept_[0])


def accuracy(X, y, w_raw, b_raw):
    score = X @ w_raw + b_raw
    pred = (score > 0.0).astype(np.float64)
    return float((pred == y).mean())


def format_float(x):
    # Compact but high-precision C float literal.
    return "{:.8g}f".format(x)


def write_policy_header(w_raw, b_raw, acc_train, acc_test, n_train, n_test):
    wa, wd, wlr, werr = w_raw
    lines = []
    lines.append("#ifndef POLICY_H")
    lines.append("#define POLICY_H")
    lines.append("")
    lines.append("/*")
    lines.append(" * Learned sensor-switching policy for the line-following robot.")
    lines.append(" *")
    lines.append(" * These coefficients are produced OFFLINE by analysis/train_policy.py and")
    lines.append(" * written in automatically by that script. Do not edit them by hand; re-run")
    lines.append(" *   python analysis/train_policy.py")
    lines.append(" * to regenerate this header from the training data.")
    lines.append(" *")
    lines.append(" * The policy is a logistic regression over four cheap per-tick features:")
    lines.append(" *   var_a      analog average variance")
    lines.append(" *   var_d      digital average variance")
    lines.append(" *   log_ratio  log((var_a+eps)/(var_d+eps)), the scale-invariant compare")
    lines.append(" *   abs_err    magnitude of the recent line-position error, |line_error|")
    lines.append(" *")
    lines.append(" * score = BIAS + W_VARA*var_a + W_VARD*var_d")
    lines.append(" *              + W_LRATIO*log_ratio + W_ERR*abs_err")
    lines.append(" * probability of preferring the digital array = sigmoid(score).")
    lines.append(" * preferDigital() returns true when that probability exceeds 0.5.")
    lines.append(" *")
    lines.append(" * The offline fit standardized the features; that standardization is folded")
    lines.append(" * into the constants below, so the device applies RAW features directly with")
    lines.append(" * no per-feature mean or scale needed on-board. Cost per call: one divide,")
    lines.append(" * one logf, a few multiply-adds (plus one expf only if the probability form")
    lines.append(" * is used; preferDigital() needs just the sign of the score).")
    lines.append(" *")
    lines.append(" * Training data is SYNTHETIC (see train_policy.py) until real labeled runs")
    lines.append(" * exist. Coefficients are reproducible with np.random.default_rng(0).")
    lines.append(" *")
    lines.append(" * Fit quality on the synthetic set:")
    lines.append(" *   train accuracy = {:.4f}  (n = {})".format(acc_train, n_train))
    lines.append(" *   test  accuracy = {:.4f}  (n = {})".format(acc_test, n_test))
    lines.append(" */")
    lines.append("")
    lines.append("#include <math.h>   // logf, expf")
    lines.append("")
    lines.append("// Guard added inside the ratio so a near-zero variance cannot divide by")
    lines.append("// zero or hit log(0). MUST match VAR_EPS in analysis/train_policy.py.")
    lines.append("static const float POLICY_VAR_EPS = {};".format(format_float(VAR_EPS)))
    lines.append("")
    lines.append("// Raw-feature logistic-regression coefficients (standardization folded in).")
    lines.append("static const float POLICY_BIAS     = {};".format(format_float(b_raw)))
    lines.append("static const float POLICY_W_VARA   = {};".format(format_float(wa)))
    lines.append("static const float POLICY_W_VARD   = {};".format(format_float(wd)))
    lines.append("static const float POLICY_W_LRATIO = {};".format(format_float(wlr)))
    lines.append("static const float POLICY_W_ERR    = {};".format(format_float(werr)))
    lines.append("")
    lines.append("// Flat coefficient array {bias, var_a, var_d, log_ratio, abs_err}, handy for")
    lines.append("// logging or bulk copies. Kept in sync with the named constants above.")
    lines.append("static const float POLICY_COEFFS[5] = {")
    lines.append("  {}, {}, {}, {}, {}".format(
        format_float(b_raw), format_float(wa), format_float(wd),
        format_float(wlr), format_float(werr)))
    lines.append("};")
    lines.append("")
    lines.append("// Linear logistic score. A positive score favors the digital array.")
    lines.append("// var_a, var_d are the two arrays' average variances this tick; line_err is")
    lines.append("// the recent line-position error in [-1, 1].")
    lines.append("static inline float policyScore(float var_a, float var_d, float line_err) {")
    lines.append("  float log_ratio = logf((var_a + POLICY_VAR_EPS) / (var_d + POLICY_VAR_EPS));")
    lines.append("  float abs_err   = (line_err < 0.0f) ? -line_err : line_err;")
    lines.append("  return POLICY_BIAS")
    lines.append("       + POLICY_W_VARA   * var_a")
    lines.append("       + POLICY_W_VARD   * var_d")
    lines.append("       + POLICY_W_LRATIO * log_ratio")
    lines.append("       + POLICY_W_ERR    * abs_err;")
    lines.append("}")
    lines.append("")
    lines.append("// Probability in [0, 1] that the digital array is the one to trust.")
    lines.append("static inline float policyProbDigital(float var_a, float var_d, float line_err) {")
    lines.append("  return 1.0f / (1.0f + expf(-policyScore(var_a, var_d, line_err)));")
    lines.append("}")
    lines.append("")
    lines.append("// Hard decision: true = prefer the digital array, false = prefer analog.")
    lines.append("// Equivalent to policyProbDigital(...) > 0.5 but skips the sigmoid.")
    lines.append("static inline bool preferDigital(float var_a, float var_d, float line_err) {")
    lines.append("  return policyScore(var_a, var_d, line_err) > 0.0f;")
    lines.append("}")
    lines.append("")
    lines.append("#endif  // POLICY_H")
    lines.append("")

    with open(POLICY_H, "w") as f:
        f.write("\n".join(lines))


def main():
    rng = np.random.default_rng(0)

    n_train, n_test = 40000, 20000
    X_train, y_train = synthesize(rng, n_train)
    X_test, y_test = synthesize(rng, n_test)

    # Standardize using train statistics.
    mu = X_train.mean(axis=0)
    sd = X_train.std(axis=0)
    sd[sd == 0.0] = 1.0
    Xs_train = (X_train - mu) / sd

    # Prefer sklearn if present; otherwise use the hand-rolled fit.
    res = try_sklearn(Xs_train, y_train)
    if res is not None:
        w_std, b_std = res
        backend = "sklearn"
    else:
        w_std, b_std = fit_logreg(Xs_train, y_train)
        backend = "numpy-gd"

    # Fold standardization into raw-feature weights:
    #   z_i = (x_i - mu_i) / sd_i
    #   score = b_std + sum_i w_std_i * z_i
    #         = (b_std - sum_i w_std_i * mu_i / sd_i) + sum_i (w_std_i / sd_i) x_i
    w_raw = w_std / sd
    b_raw = b_std - np.sum(w_std * mu / sd)

    acc_train = accuracy(X_train, y_train, w_raw, b_raw)
    acc_test = accuracy(X_test, y_test, w_raw, b_raw)

    print("Learned switching policy (backend: {})".format(backend))
    print("=" * 60)
    print("features (order): {}".format(", ".join(FEATURES)))
    print("standardized weights: {}".format(
        ", ".join("{:+.4f}".format(v) for v in w_std)))
    print("standardized bias:    {:+.4f}".format(b_std))
    print("-" * 60)
    print("raw-feature coefficients (folded, device applies these):")
    print("  BIAS     = {:+.8g}".format(b_raw))
    print("  W_VARA   = {:+.8g}".format(w_raw[0]))
    print("  W_VARD   = {:+.8g}".format(w_raw[1]))
    print("  W_LRATIO = {:+.8g}".format(w_raw[2]))
    print("  W_ERR    = {:+.8g}".format(w_raw[3]))
    print("-" * 60)
    # Confirm the exported decision is monotonic in the variance difference across
    # the realistic range: raising var_a (var_d fixed) must never lower the score,
    # and raising var_d (var_a fixed) must never raise it. The log_ratio term
    # dominates the raw var_a/var_d terms over these magnitudes.
    def raw_score(va, vd, e=0.0):
        lr = np.log((va + VAR_EPS) / (vd + VAR_EPS))
        return b_raw + w_raw[0] * va + w_raw[1] * vd + w_raw[2] * lr + w_raw[3] * abs(e)
    grid = np.logspace(-5, -2, 200)
    mono_up = all(raw_score(va, 1e-4) < raw_score(va + 1e-5, 1e-4) for va in grid)
    mono_dn = all(raw_score(1e-4, vd) > raw_score(1e-4, vd + 1e-5) for vd in grid)
    print("monotonicity in the variance difference (var 1e-5 .. 1e-2):")
    print("  score increases with var_a (var_d fixed): {}".format(mono_up))
    print("  score decreases with var_d (var_a fixed): {}".format(mono_dn))
    print("-" * 60)
    print("train accuracy = {:.4f}  (n = {})".format(acc_train, n_train))
    print("test  accuracy = {:.4f}  (n = {})".format(acc_test, n_test))
    print("-" * 60)

    # C array for the maintainer, in {bias, var_a, var_d, log_ratio, abs_err} order.
    c_vals = [b_raw, w_raw[0], w_raw[1], w_raw[2], w_raw[3]]
    print("C coefficient array {bias, var_a, var_d, log_ratio, abs_err}:")
    print("  static const float POLICY_COEFFS[5] = {")
    print("    " + ", ".join(format_float(v) for v in c_vals))
    print("  };")

    write_policy_header(w_raw, b_raw, acc_train, acc_test, n_train, n_test)
    print("-" * 60)
    print("wrote coefficients into: {}".format(POLICY_H))


if __name__ == "__main__":
    main()
