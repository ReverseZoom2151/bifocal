# Statistical analysis of the Bifocal trials

This document quantifies, with confidence intervals and significance tests, the
central claim in the paper and README: that variance-based switching improves
sensor reliability but reduces heading smoothness. The original write-up gave
only point estimates (for example "curve variance drops by about 75 percent")
with no uncertainty and no significance testing. Here every number carries a 95
percent bootstrap confidence interval, and each switching-vs-baseline comparison
carries a non-parametric test.

All numbers are produced by `analysis/statistics.py`, which reads the raw CSVs
in `data/results/` directly. Re-run it with `python analysis/statistics.py`; it
also writes the figure `gallery/statistics.png`. Nothing below is entered by
hand.

## Method

- Parsing is identical to `analysis/make_gallery.py` and
  `analysis/sensor_noise.py`: PuTTY `=~` header lines are skipped, lines
  carrying the `Variance` label are skipped, each kept row is 5 comma-separated
  per-sensor variances, and the theta dumps (delimiter-free, reprinting a
  growing history) are reduced to the single longest run via a regex over signed
  decimals, dropping the `-1.0` sentinel and any `|x| > 100` PuTTY artifact.
- Variance-stability metric: for each logged row, the average per-sensor
  variance is the mean across the 5 sensors. This is exactly the scalar the
  firmware compares when it decides which array to trust. Each method and track
  therefore yields a 1-D sample with one value per row. We report the median,
  mean, and interquartile range (IQR). The median is the headline statistic
  because the distributions are dominated by rare large spikes (the means are
  far above the medians).
- Heading-smoothness metric: from the longest theta run per method on the
  straight track, roughness is the standard deviation of theta and the mean
  absolute step `|dtheta|` between consecutive samples. Lower is smoother.
- Uncertainty: every metric gets a 95 percent percentile bootstrap confidence
  interval from 10000 resamples of the underlying rows, with a fixed seed
  (`np.random.default_rng(0)`) for reproducibility.
- Pairwise tests: switching vs analog and switching vs digital on the average
  variance, for each track. We report the difference in medians
  (median(switching) minus median(baseline); negative favours switching) with a
  bootstrap CI, and a Mann-Whitney U test (two-sided). scipy is present, so the
  Mann-Whitney U test is used; if scipy were absent the script falls back to a
  permutation p-value it implements. Significance is judged at alpha = 0.05.

## 1. Variance stability

Average per-sensor variance per logged row (lower is steadier). The 95 percent
CI is for the median.

### Straight line

| Method | n | Median | Mean | IQR | 95% CI of median |
|--------|---|--------|------|-----|------------------|
| Analog | 490 | 0.000059 | 0.000704 | 0.000351 | [0.000007, 0.000089] |
| Digital | 493 | 0.000041 | 0.000140 | 0.000098 | [0.000031, 0.000050] |
| Switching | 275 | 0.000038 | 0.000416 | 0.000478 | [0.000015, 0.000082] |

### Curve

| Method | n | Median | Mean | IQR | 95% CI of median |
|--------|---|--------|------|-----|------------------|
| Analog | 500 | 0.000485 | 0.005727 | 0.004953 | [0.000330, 0.000764] |
| Digital | 495 | 0.000074 | 0.000464 | 0.000493 | [0.000048, 0.000117] |
| Switching | 230 | 0.000128 | 0.000544 | 0.000643 | [0.000086, 0.000190] |

Notes:

- The means sit far above the medians for every method (most extreme for analog
  on curves: mean 0.005727 vs median 0.000485, about 12x), confirming the
  spike-dominated distributions the README warns about. Medians are the honest
  summary.
- Analog is the noisiest on curves by a wide margin. Digital is the steady
  baseline. Switching lands between them: much steadier than analog, slightly
  above digital on curves.

## 2. Heading smoothness (straight line, preliminary)

Roughness from the longest theta run per method. The runs are short and uneven,
and switching has roughly three times fewer samples, so treat this as
directional only.

| Method | n | std(theta) | 95% CI | mean abs dtheta | 95% CI |
|--------|---|-----------|--------|-----------------|--------|
| Analog | 1170 | 0.0620 | [0.0592, 0.0646] | 0.0343 | [0.0317, 0.0368] |
| Digital | 1270 | 0.0784 | [0.0753, 0.0814] | 0.0827 | [0.0787, 0.0868] |
| Switching | 378 | 0.2052 | [0.1961, 0.2127] | 0.1923 | [0.1740, 0.2111] |

On both roughness measures the ordering is analog (smoothest) < digital <
switching (roughest), and the confidence intervals do not overlap between
methods. Switching's std is about 3.3x analog's and its mean absolute step is
about 5.6x analog's. The CIs are narrow only because they describe dispersion
within a single run; they do not capture run-to-run variability, of which there
is exactly one run per method here. See the caveats.

## 3. Pairwise comparison (switching vs baseline)

Difference in medians is median(switching) minus median(baseline); a negative
value means switching is steadier. The test is Mann-Whitney U (two-sided).

| Track | Comparison | median(sw) | median(base) | Diff in medians | 95% CI of diff | U | p | Significant (a=0.05) |
|-------|-----------|-----------|--------------|-----------------|----------------|---|---|----------------------|
| Straight | sw vs analog | 0.000038 | 0.000059 | -0.000021 | [-0.000060, +0.000046] | 76994.5 | 1.04e-03 | yes |
| Straight | sw vs digital | 0.000038 | 0.000041 | -0.000003 | [-0.000028, +0.000043] | 66861.0 | 7.53e-01 | no |
| Curve | sw vs analog | 0.000128 | 0.000485 | -0.000356 | [-0.000652, -0.000180] | 42448.5 | 1.30e-08 | yes |
| Curve | sw vs digital | 0.000128 | 0.000074 | +0.000054 | [-0.000001, +0.000124] | 63613.0 | 1.08e-02 | yes |

Reading the table:

- Switching vs analog on curves is the strongest and clearest result: the median
  average variance is about 3.8x lower under switching, the difference is
  -0.000356 with a CI of [-0.000652, -0.000180] that stays well below zero, and
  the test is highly significant (p about 1.3e-08). This is the reliability win
  the paper claims, and it holds up.
- Switching vs analog on the straight line is significant by Mann-Whitney U
  (p about 1.0e-03), but the median-difference CI [-0.000060, +0.000046] crosses
  zero. These are not contradictory: Mann-Whitney U tests the whole distribution
  (stochastic dominance), and the straight-line analog sample is spike-heavy, so
  switching wins across the distribution while the two medians are close and hard
  to separate. The honest statement is: switching helps on straights, but the
  size of the median improvement is uncertain.
- Switching vs digital on the straight line is not significant (p about 0.75);
  the two are statistically indistinguishable. This matches the README's "ties"
  verdict.
- Switching vs digital on curves is significant (p about 0.011) but in the
  wrong direction for switching: the median is about 0.000054 higher under
  switching, and the CI [-0.000001, +0.000124] is almost entirely above zero.
  Digital alone is slightly steadier than switching on curves. This is the
  "slightly worse on curves" the README notes, and it is a genuine, if small,
  effect.

## Interpretation, tied back to the paper's claim

The paper's two-part claim survives scrutiny, with nuance:

- Reliability up: yes, but specifically relative to analog. Switching removes
  analog's large curve spikes, cutting median average variance about 3.8x on
  curves with a tight, clearly-below-zero CI and p about 1e-08. Against digital,
  switching only ties (straight) or is marginally worse (curve). The value of
  switching is insuring against analog's worst case, not beating the digital
  baseline.
- Smoothness down: yes, directionally. On the single available straight run per
  method, switching's heading roughness is about 3 to 6x that of analog on both
  measures, with non-overlapping within-run CIs. But this rests on one run per
  method with uneven and small n, so it is indicative, not established.

## Caveats (do not overclaim)

- Small and uneven samples. Switching has far fewer usable variance rows (275
  straight, 230 curve) than analog or digital (about 490 to 500 each), because
  the switching log interleaves analog-mode and digital-mode rows. Comparisons
  are unbalanced.
- Thin heading data. There is effectively one theta run per method, and
  switching's run has roughly a third of the samples (378 vs about 1170 to
  1270). The within-run bootstrap CIs quantify dispersion inside that one run
  only; they say nothing about run-to-run variability, which is the variability
  that actually matters for a smoothness claim. No significance test is run on
  theta for this reason.
- Spike-dominated distributions. Means are pulled far above medians, so every
  variance conclusion leans on the median. A mean-based analysis would tell a
  different and misleading story.
- The theta metric is a dispersion sample of visited headings recovered from an
  accumulating, delimiter-free serial dump, not a clean trajectory. Units and
  absolute scale should not be over-interpreted; only the ordering between
  methods is used.
- Multiple comparisons. Four pairwise tests are reported without a
  multiple-comparison correction. The two strongest results (curve sw vs analog,
  and the non-significant straight sw vs digital) are robust to any reasonable
  correction; the two borderline results (straight sw vs analog, curve sw vs
  digital) should be read as suggestive.
- Reproducibility. All CIs and p-values use a fixed seed
  (`np.random.default_rng(0)`) and 10000 bootstrap resamples, so the numbers
  above are reproducible exactly by re-running `analysis/statistics.py`.
