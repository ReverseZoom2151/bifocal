# Sensor-disagreement feature detector

`firmware/line_following/disagreement.h`

## Concept

Bifocal reads the same line two ways: an analog pipeline
(`AnalogLineSensors_c`) and a digital RC-decay pipeline
(`DigitalLineSensors_c`). Each produces a `calibrated[5]` array normalised to
roughly 0..1, and a single line position can be derived from either with
`linePosition5()` in `steering.h`.

For most of a straight, clean line the two pipelines agree closely: they are
looking at the same surface through the same optics. When they disagree
sharply, that disagreement is itself a signal. It tends to coincide with:

- a line edge (the moment a sensor crosses the black/white boundary, where the
  two sampling methods respond with slightly different timing and gain),
- a junction or branch (extra line area appears off to one side),
- a local surface anomaly (tape seam, glue line, glare, a scuff).

The detector turns the analog-vs-digital disagreement into a per-tick number,
tracks a short rolling baseline of what "normal" disagreement looks like, and
raises event flags when the current tick spikes above that baseline. It needs
no new hardware and does not modify the sensor pipelines.

This header is self-contained and is deliberately NOT wired into
`line_following.ino`. It is a building block you can log, gate on, or feed into
recovery logic.

## The metric

Two component measures are combined per tick, both in comparable 0..1 units:

1. Per-sensor mean absolute difference (`disagreementMAD`): the average over
   the five sensors of `|analog[i] - digital[i]|`. This is sensitive to any
   local shape difference between the two arrays, even one that does not move
   the overall line position.

2. Line-position disagreement (`disagreementPosition`): `|posA - posD|` where
   each position comes from `linePosition5()` (range [-1, 1]). This captures
   only the disagreement that actually moves the quantity the controller steers
   on.

The combined tick metric is a weighted blend:

```
metric = w * MAD + (1 - w) * positionDisagreement
```

The weight `w` defaults to `DISAGREE_MAD_WEIGHT` (0.5) and can be set at runtime
with `setMadWeight()`. There is also a free function `disagreementMetric()` that
uses the compile-time weight for callers that do not need the stateful detector.

## Rolling baseline

`DisagreementDetector_c` keeps a small fixed ring buffer
(`DISAGREE_BASELINE_WINDOW`, default 16 ticks) of recent metric values and a
running sum for an O(1) mean. Call `update(analogCal, digitalCal)` once per
control loop. Each call:

1. computes the combined metric for the current tick,
2. compares it against the current baseline (which excludes this tick, so a
   lone spike is measured against recent normal),
3. classifies the tick, then folds the metric into the baseline.

The first `update()` seeds the whole window with the first metric so tick one
does not false-fire against a zero baseline.

Because every tick, including spikes, is folded into the baseline, a
persistently noisy surface raises the baseline and the event flag clears on its
own. The detector adapts: it reports genuine departures from recent normal
rather than an absolute disagreement level.

## Interpreting the flags

- `lastMetric()` current tick metric.
- `baseline()` the rolling mean the tick was compared against.
- `lastExcess()` how far the tick sat above the baseline (may be negative).
- `level()` one of `DISAGREE_NONE`, `DISAGREE_ELEVATED`, `DISAGREE_STRONG`.
- `isEdgeOrJunction()` true for elevated or strong: a candidate feature worth
  reacting to.
- `isStrong()` true only for strong: high confidence it is a real edge, branch,
  or anomaly rather than sensor noise.

Classification uses two margins above the baseline:

- `DISAGREE_ELEVATED_MARGIN` (default 0.10): excess at or above this is at least
  elevated.
- `DISAGREE_STRONG_MARGIN` (default 0.25): excess at or above this is strong.

Both are configurable per instance with `setElevatedMargin()` and
`setStrongMargin()`, or globally by defining the macros before the include.

## Wiring it into the controller

The detector is intentionally standalone. Some ways it could be used:

- Logging and tuning. Print `lastMetric()`, `baseline()`, and `level()` next to
  the line error over the serial link. This is the recommended first step: it
  lets you see what the disagreement actually does at edges and junctions on
  your track before you trust a threshold.

- Trigger recovery or junction handling. When `isEdgeOrJunction()` fires, hand
  off to whatever branch or intersection routine the robot has, or slow down and
  re-acquire.

- Gate the analog/digital switch decision. A strong-disagreement tick is a poor
  moment to trust either single reading, so the switching logic can hold its
  current choice until the disagreement subsides.

A typical call site would run one `update()` per loop after both pipelines have
refreshed their `calibrated[]` arrays, then branch on the flags.

## Honest caveat on tuning

The default margins are starting guesses, not calibrated values. Without
labelled junction data (recorded runs where you know exactly when the robot was
over an edge, a branch, or clean line) there is no way to set the thresholds
optimally from a desk. The metric and baseline machinery is sound, but the two
margins WILL need tuning on the actual robot and track: log a few runs, look at
where the metric spikes relative to the baseline at known features, and set the
elevated and strong margins from those observed excesses. Sensor gain trim,
lighting, and surface all shift the numbers, so re-check after any of those
change.
