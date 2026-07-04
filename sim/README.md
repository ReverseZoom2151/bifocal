# Bifocal replay / simulation harness

An offline, host-only (no hardware) replay harness that runs all four
line-following strategies on ONE identical synthetic input stream, so they can
be compared apples to apples.

## Why this exists

The original study compared the analog, digital, and variance-based switching
strategies on separate physical runs. Those runs were not comparable: different
headings, unequal sample counts, and different noise realisations each time.
Any difference in the numbers could have come from the track or the run rather
than from the control law.

This harness removes that weakness. It generates a single ground-truth line
trajectory and a single deterministic noise realisation, then drives every
strategy through the exact same inputs. The only thing that differs between the
strategies is the control law, so the tracking-accuracy vs steering-smoothness
trade-off becomes directly measurable.

## What it reuses from the firmware

- `firmware/line_following/steering.h`: `linePosition5()` (the weighted
  5-sensor line position in [-1, 1]) and `PID_c` (the turn-term controller).
- `tests/arduino_stub.h`: only for the Arduino `constrain()` macro that
  `PID_c::update()` uses, so `steering.h` compiles on the host.

The strategy logic (analog / digital / switching with hysteresis + dwell /
inverse-variance fusion) is replicated from `line_following.ino`.

## What it does, step by step

1. Ground truth: a sinusoid (gentle curve) plus additive step segments (abrupt
   lateral offsets, like sharp corners), giving both straight and curved
   sections. See `groundTruth()`.
2. Shared noise: from a fixed PRNG seed (`std::mt19937`, default 12345), it
   pre-generates ONE noise/spike realisation for all ticks. Every strategy is
   disturbed by the identical random sequence.
3. Sensor synthesis, per strategy, per tick: two 5-element calibrated arrays
   centred on the line position the robot currently sees.
   - ANALOG view: continuous reflectance bump, more Gaussian noise, occasional
     large spikes (higher resolution but noisier).
   - DIGITAL view: same bump quantized to {0, 1} at a threshold, less noise,
     rare stuck-high spikes (coarser but steadier).
4. Rolling variance: mean over the 5 sensors of each sensor's variance across a
   short window of recent ticks (`VAR_WINDOW`), the same reduction the firmware
   `calculateAverageVariance()` performs.
5. Four controllers run independently over the same inputs. Each reuses
   `linePosition5()` + a `PID_c` to produce a turn term, which drives a simple
   1D robot model (the turn steers the robot laterally toward the line).

## Metrics printed

- `steerRMS`: RMS of the steering derivative (turn-command jerkiness). Lower is
  smoother.
- `IAE`: integral of absolute tracking error over the run. Lower tracks better.
- `meanAbsErr`: mean absolute tracking error per tick.
- `finalErr`: absolute tracking error on the last tick.
- `switch/100`: array switches per 100 ticks (switching strategy only). The
  hysteresis + dwell debounce is what keeps this low; without it the switching
  strategy would flip arrays far more often and add steering-derivative energy.

## Build and run

Requires `g++` (C++11) and `make`.

```
make        # build
make run    # build and run with the default fixed seed
make clean  # remove the binary
```

Run with a different but still deterministic seed:

```
./replay 999
```

The seed is a compile-time default or a CLI argument. Nothing depends on
wall-clock time, so every run is reproducible.

## How to read the output

Compare the columns across rows. Expect a trade-off: the noisier analog-only
input tracks worst; digital-only can track tightly but its quantized readings
make the steering command jumpier (higher `steerRMS`); switching sits between
them and its `switch/100` count stays modest thanks to the debounce; fusion
tends to give the smoothest steering by blending both arrays instead of making
a hard pick. The exact numbers depend on the seed, but the ordering and the
accuracy-vs-smoothness story are the point.
