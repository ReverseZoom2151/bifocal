# Learned sensor-switching policy

## What this is

The line-following firmware runs one of four strategies (analog only, digital
only, variance-based switching, inverse-variance fusion). The switching strategy
in `firmware/line_following/line_following.ino` decides which IR array to trust
by comparing the two arrays' average reading variances against a single
hand-tuned constant:

```
// METHOD_SWITCHING, inside computeControl()
if (g_var_d + SWITCH_MARGIN < g_var_a) switch_choice = 1;  // go digital
if (g_var_a + SWITCH_MARGIN < g_var_d) switch_choice = 0;  // go analog
```

`SWITCH_MARGIN` (0.0005) is a guess. It is a raw variance difference, which is
awkward: the two variances span several orders of magnitude, so a fixed
difference means very different things at 1e-5 than at 1e-2.

This document describes an optional drop-in replacement: a tiny logistic
regression trained offline in Python and applied on-device from a small header.
It turns the guess into learned coefficients while staying cheap enough for the
32U4 (a divide, a `logf`, and a few multiply-adds per tick). It is deliberately
NOT wired into `line_following.ino`; that wiring is left to the maintainer and
sketched below.

Files:

- `analysis/train_policy.py` trains the policy and writes the coefficients.
- `firmware/line_following/policy.h` applies them on-device.
- `tests/test_policy.cpp` host-side unit tests.

## The model

A logistic regression over four features that are all already available every
control tick in `computeControl()`:

| feature     | meaning                                         | source in the .ino     |
| ----------- | ----------------------------------------------- | ---------------------- |
| `var_a`     | analog array average variance                   | `g_var_a`              |
| `var_d`     | digital array average variance                  | `g_var_d`              |
| `log_ratio` | `log((var_a + eps) / (var_d + eps))`            | derived from the above |
| `abs_err`   | magnitude of the recent line error, `|line_err|`| `fabs(e_a)` or `g_line_error` |

The score is a plain linear combination:

```
score = BIAS + W_VARA*var_a + W_VARD*var_d + W_LRATIO*log_ratio + W_ERR*abs_err
```

The probability of preferring the digital array is `sigmoid(score)`, and the
hard decision `preferDigital()` is simply `score > 0`.

### Why the log ratio matters

Comparing two variances that range over orders of magnitude is a ratio
question, not a difference question. A raw `var_a - var_d` difference (which is
what `SWITCH_MARGIN` uses) was tried first as the model's main feature and
plateaued around 0.69 accuracy on the synthetic set, because the same "digital
is 2x steadier" situation looks tiny at 1e-5 and huge at 1e-2. Swapping in the
scale-invariant `log_ratio` lifts accuracy to about 0.83, matching the
oracle-style rule "prefer whichever array truly has the lower noise". `var_a`
and `var_d` are kept as raw features so isolated absolute spikes can still carry
a little weight; `abs_err` gets a small weight and does not overturn a clear
variance decision.

### Standardization is folded in

The offline fit standardizes each feature (subtract mean, divide by standard
deviation) for good conditioning. Those per-feature means and scales are then
folded back into the exported weights, so the device applies the transform

```
z_i   = (x_i - mu_i) / sd_i
score = b_std + sum_i w_std_i * z_i
      = (b_std - sum_i w_std_i * mu_i / sd_i) + sum_i (w_std_i / sd_i) * x_i
```

as raw multiply-adds. No means or scales need to live on the microcontroller.

## Honest caveat: the training data is synthetic

There is no labeled ground truth yet for "which array was actually right" on a
given tick, so `train_policy.py` synthesizes the training set. For each sample
it draws a true noise level for each array from log-normal distributions matched
to the ranges in `data/results` (analog median around 1e-4 with occasional large
spikes, digital tighter around 5e-5), corrupts them with a measurement-noise
factor to model the finite rolling-variance window, labels each sample by which
array had the lower TRUE noise, and flips 8 percent of labels to model
real-world label noise. The reported accuracy (about 0.83 train and test) is
therefore accuracy against this synthetic generator, not against the track.

Treat the shipped coefficients as a principled default that behaves like the
old margin compare but scale-correctly, not as a validated learned controller.
Replace the synthetic generator with real labeled runs as soon as they exist
(see below), then retrain. The everything-else (feature set, folding,
on-device code, tests) stays the same.

## How to retrain

From the repo root:

```
python analysis/train_policy.py
```

This prints the standardized weights, the folded raw-feature coefficients, a
monotonicity check, and train/test accuracy, then overwrites the coefficient
block in `firmware/line_following/policy.h`. The run is fully seeded with
`np.random.default_rng(0)`, so it is reproducible and produces identical
coefficients every time. It uses `scikit-learn` if it happens to be installed
and otherwise falls back to a hand-rolled numpy gradient descent that reaches
the same accuracy, so the only hard dependency is numpy.

To move off synthetic data, replace `synthesize()` with a loader that reads real
per-tick features and labels, keeping the same 4-column feature order
(`var_a, var_d, log_ratio, abs_err`) and the same 1 = digital label convention.
A practical way to get labels: run both arrays in parallel (the firmware already
logs `var_a` and `var_d` every tick), and label each tick by which array's line
error better matched the true track position, or by which array a human judged
correct from the logged run. Everything downstream is unchanged.

## How to wire it into the controller

`policy.h` is standalone and AVR-safe: it includes only `<math.h>` and needs no
`Arduino.h`. To use it, include it near the other headers in
`line_following.ino`:

```
#include "policy.h"
```

Then, inside `computeControl()`, replace the fixed-margin compare in the
`METHOD_SWITCHING` branch. The current code is:

```
if (millis() - switch_last_ts >= SWITCH_DWELL_MS) {
  if (switch_choice == 0) {
    if (g_var_d + SWITCH_MARGIN < g_var_a) { switch_choice = 1; switch_last_ts = millis(); }
  } else {
    if (g_var_a + SWITCH_MARGIN < g_var_d) { switch_choice = 0; switch_last_ts = millis(); }
  }
}
```

The learned version keeps the dwell-time debounce but asks the policy for the
decision. `e_a` is the analog line error already computed a few lines above:

```
if (millis() - switch_last_ts >= SWITCH_DWELL_MS) {
  bool want_digital = preferDigital(g_var_a, g_var_d, e_a);
  int  desired = want_digital ? 1 : 0;
  if (desired != switch_choice) {
    switch_choice  = desired;
    switch_last_ts = millis();
  }
}
```

The debounce (`SWITCH_DWELL_MS`) and the rest of the control step are untouched.
If you want hysteresis analogous to the old margin, gate the switch on the
probability instead of the sign, for example only switch to digital when
`policyProbDigital(g_var_a, g_var_d, e_a) > 0.6` and back to analog when it drops
below 0.4; `preferDigital()` alone corresponds to a plain 0.5 threshold.

`SWITCH_MARGIN` becomes unused once the policy is in charge and can be removed,
or kept for easy A/B comparison against `METHOD_SWITCHING` as it stands.

## On-device cost

Per call, `preferDigital()` does one add of `eps`, one divide, one `logf`, four
multiplies, four adds, and a sign test. `policyProbDigital()` adds one `expf`.
No allocation, no STL, no floating-point surprises beyond what the existing
variance and PID math already use.
