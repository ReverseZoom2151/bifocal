# Bifocal parameter sweep and Monte Carlo study

This document explains the parameter sweep that sits on top of the offline
replay harness, how the Monte Carlo confidence intervals are computed, how to
read the Pareto plot, and which region of (switch margin, switch dwell) looks
best to try on the real robot.

Everything here is deterministic. The sweep uses a fixed list of seeds baked
into the source, so re-running it reproduces the same CSV and the same figures.

## What the sweep varies

The reusable simulation core lives in `sim/harness.h` behind one entry point:

```
bifocal::TrialResult runTrial(const bifocal::Params& p, unsigned int seed);
```

`runTrial()` drives all four strategies (analog-only, digital-only, variance
switching, inverse-variance fusion) over ONE identical synthetic input stream
built from the seed, and returns per-strategy metrics. `Params` carries the
knobs the sweep moves:

- `switchMargin`: the hysteresis margin for the switching strategy. The
  switching controller only changes array when the candidate array's rolling
  variance beats the current array's variance by at least this margin. A margin
  of 0 switches on any advantage; a large margin switches only on a big one.
- `switchDwell`: the debounce. The minimum number of ticks that must pass
  between switches. One tick is 100 ms.
- `Kp`: the PID proportional gain (swept as a third dimension).

The track, the noise/spike model, `Ki`, `Kd`, the variance window, and the
robot model are held fixed. Holding them fixed is what makes the sweep an
apples-to-apples comparison of the control laws rather than of the tracks.

`sim/sweep.cpp` walks the grid:

- `switchMargin` in {0, 0.0002, 0.0005, 0.001, 0.002, 0.004, 0.008}
- `switchDwell` in {1, 2, 3, 5, 8, 12} ticks
- `Kp` in {0.8, 1.0, 1.2}

For each grid cell it runs `runTrial()` over the fixed seed list and writes the
mean and standard deviation of `steerRMS` and `IAE` (and the switch rate) to
`sim/sweep_results.csv`. The three parameter-independent strategies
(analog-only, digital-only, fusion) do not depend on the switch margin or dwell,
so they are recorded once per `Kp` as baseline reference rows.

Build and run it:

```
cd sim
make sweep        # writes sim/sweep_results.csv
```

## The two metrics (the two axes)

- `steerRMS`: RMS of the steering-command derivative (turn-command jerkiness).
  Lower is smoother. This is the steering-smoothness axis.
- `IAE`: integral of the absolute tracking error over the run. Lower tracks
  better. This is the tracking-accuracy axis.

A controller that tracks tightly but with a jumpy steering command scores well
on IAE and poorly on steerRMS, and vice versa. The whole point is the trade-off
between the two, so neither number alone tells the story.

## Monte Carlo confidence intervals

Each grid cell is evaluated over a fixed list of 40 seeds
(`SEED_BASE = 1000`, seeds 1000 through 1039). Every seed produces a different
noise/spike realisation on the same track, so the 40 results are a Monte Carlo
sample of how the strategy behaves under different disturbance draws.

For each metric the sweep records the sample mean and the sample standard
deviation (n-1 denominator) across those 40 seeds. `analysis/pareto.py` turns
the standard deviation into a 95 percent confidence interval for the mean:

```
CI95 half-width = 1.96 * std / sqrt(n)     with n = 40
```

These are the error bars in `gallery/montecarlo.png`. They describe the
uncertainty in the estimated mean metric, not the spread of a single run. Tight
bars (as seen here) mean the ranking between strategies is not an artefact of
one lucky or unlucky seed. The seeds are fixed, so the intervals are
reproducible.

## How to read the Pareto plot

`gallery/pareto.png` is a scatter of IAE (y, tracking accuracy) against steerRMS
(x, steering smoothness) at `Kp = 1.0`. Lower-left is better on both axes.

- The faint green points are the swept switching cells, one per
  (margin, dwell) pair.
- The three diamonds are the baseline strategies (analog-only, digital-only,
  fusion).
- The red line is the Pareto frontier: the set of non-dominated points. A point
  is on the frontier if no other point is at least as good on both axes and
  strictly better on one. Points on the frontier represent the achievable
  trade-offs; everything above and to the right of it is dominated.

In this study the global frontier is spanned by fusion (the smoothest steering,
lowest steerRMS, with a low IAE) and digital-only (the best tracking, lowest
IAE, but the jerkiest steering). Fusion dominates most of the switching grid on
smoothness while tracking nearly as well, which is why the switching cells sit
in the interior rather than on the frontier. That is an honest result of the
harness, not a tuning failure: blending both arrays every tick avoids the
steering-derivative energy that a hard array switch injects.

The switching strategy is still the right choice when a hard, explainable array
selection is wanted on the robot (fusion needs both pipelines live and trusted
every tick). The sweep tells you how to tune that selection.

## Best (margin, dwell) region to try on the robot

Reading the switching cells at `Kp = 1.0`:

- Dwell dominates the outcome. Low dwell (1 to 2 ticks) is best on both axes.
  Raising the dwell delays beneficial switches, which both raises IAE and, once
  the switch finally fires, adds a larger steering-derivative jump, raising
  steerRMS. Dwell 8 and 12 are clearly worse.
- Margin trades switch chatter against tracking. Small margin (0 to 0.0005)
  gives the lowest IAE (about 6.40) but the highest switch rate (about
  15 switches per 100 ticks). Raising the margin toward 0.004 to 0.008 cuts the
  switch rate to about 7 to 8 per 100 ticks and lowers steerRMS slightly, at the
  cost of IAE rising toward 7.05.

Recommended starting region for the robot:

- `switchDwell = 1 to 2 ticks` (100 to 200 ms).
- `switchMargin around 0.001 to 0.002`.

That region (for example margin 0.001, dwell 1) gives steerRMS about 0.38 and
IAE about 6.41 with a switch rate near 13 to 15 per 100 ticks: close to the best
achievable tracking for the switching strategy while keeping the steering
reasonably smooth. If switch chatter is a practical problem on the hardware
(motor wear, audible relay-like behaviour), move the margin up to about 0.004 at
dwell 1: that roughly halves the switch rate to about 8 per 100 ticks for a
small IAE cost (about 7.05). Avoid large dwell values; they help nothing here.

The `Kp` slices (0.8, 1.0, 1.2) in the CSV move all strategies together and do
not change this margin/dwell conclusion; 1.0 is a reasonable default to keep
while tuning the switch behaviour first.

## Reproducing the figures

```
cd sim && make sweep          # writes sim/sweep_results.csv (40 seeds per cell)
python analysis/pareto.py      # writes gallery/pareto.png and gallery/montecarlo.png
```
