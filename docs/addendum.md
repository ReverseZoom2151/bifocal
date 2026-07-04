# Addendum and errata to the paper

This document records what changed in the Bifocal repository after the technical
paper (`docs/variance-based-sensor-switching-paper.pdf`) was submitted. It is an
addendum and errata, not a rewrite. The core idea from the paper is unchanged:
read each reflectance sensor two ways, measure how steady each read is with
variance, and steer with the steadier array. What follows is grounded in what is
actually in the repository now.

The headline finding from the paper still holds. Variance-based switching
improves sensor reliability but adds heading instability, an accuracy versus
smoothness trade-off rather than a one-sided win. The changes below either fix
defects in the original firmware or build out the paper's own future work
(refining the switching algorithm and adding more sophisticated filtering for
erratic data) so it can be tested on the robot.

## Corrected heading formula (erratum)

The differential-drive heading update was wrong in the original firmware. The
change in heading per control step is now:

    deltaTheta = (dR - dL) / wheelSeparation

where `dR` and `dL` are the left and right wheel displacements. The previous code
divided by twice the wheel separation, which halved every heading increment and
made the logged `theta` systematically understate how far the robot had actually
turned.

Because of this, logged `theta` values are on a different scale than in the
original runs. Re-baseline before comparing new theta values to old runs. Do not
read the paper's heading numbers and the new logs on the same axis without
accounting for this correction.

## Firmware correctness fixes

These changes removed real bugs and undefined behaviour. The control algorithm's
intended behaviour was preserved; the fixes make the firmware actually do what it
was meant to do.

- Added the missing `motors.initialise()` call, so the motors are configured
  before the control loop drives them.
- Removed a stray brace and an always-on `while(true)` block that stranded
  `loop()`, so the Arduino loop now runs as intended.
- Made `calculateVariance()` actually return its result instead of falling off
  the end of a non-void function.
- Replaced variable-length arrays (VLAs, which are not standard C++ and risk
  stack problems on the microcontroller) with fixed-size arrays.
- Fixed the analog sampling loop, which had been overwriting its accumulator each
  iteration and discarding all but the last sample instead of averaging.
- Made the analog and digital average-variance computation consistent, so the
  switch compares like with like when it decides which array is steadier.
- De-collided the two sensor headers' pin macros by giving them `A_` and `D_`
  prefixes, so the analog and digital pin definitions no longer clash.
- Fixed a `thetaLog` buffer overflow.

## New capabilities that extend the paper's method

The paper's future work was to refine the switching algorithm and add more
sophisticated filtering. The following are implemented so they can be tried on
the robot.

- Full 5-sensor weighted line position and a PID option. The paper's controller
  used two sensors; the firmware now computes a weighted position across all five
  elements (weights -2, -1, 0, 1, 2, normalised to -1 to 1) and can produce the
  turn term with either a simple proportional law or a small PID controller. The
  original 2-sensor measurement is still selectable behind `USE_FULL_POSITION`.
- Debounced switching. This targets the paper's smoothness problem directly. The
  switch now only changes array when the alternative is steadier by at least a
  hysteresis margin (`SWITCH_MARGIN`) and never more often than a minimum dwell
  time (`SWITCH_DWELL_MS`). This suppresses the per-tick flip-flopping that was
  the suspected source of the added heading wobble.
- Inverse-variance fusion (`METHOD_FUSION`) as an alternative to the hard switch.
  Instead of discretely picking one array, fusion blends the two arrays' line
  position errors weighted by inverse variance, so the steadier array simply
  counts for more and there is no discrete switch event to inject a step.
- Optional EMA low-pass filtering and outlier or median smoothing on calibrated
  readings. This is the paper's "more sophisticated filtering" future work. The
  filtering is off by default and applied symmetrically across both arrays, so
  the unfiltered pipeline is unchanged when it is not enabled.
- Non-blocking rolling variance, so variance is maintained across control ticks
  without stalling the loop.
- Adaptive speed.
- Clean, fixed-rate, timestamped CSV logging, one row per control tick
  (`t_ms,theta,method,line_error,var_a,var_d`).

## Sensor-noise finding and the gain trim

Re-analysis of the trial logs surfaced a finding not in the paper: under the
switching strategy the middle sensor (index 2) is the noisiest channel, carrying
roughly 1.75x to 2.34x the side-sensor average. This is inherited from the
digital branch. The middle sensor physically straddles the line and carries the
steepest, largest signal, and the digital calibration has the widest
normalization span there, so motion jitter is amplified most at that sensor.
Switching spends most of its time in the digital branch, so it copies that
branch's steep middle-sensor behaviour. The full investigation is in
`docs/sensor-noise.md`.

To address this, a gain-trim mechanism was added so the digital branch's raw
middle-sensor span can be brought toward the analog span, so a mode swap stops
injecting a step change at the middle sensor. The effect should be validated by
re-running `analysis/sensor_noise.py` and confirming the switching sensor-2 ratio
falls toward 1.0x.

## New tooling

- Host unit tests. The pure logic (population-variance formula, average-variance
  helper, and the differential-drive pose and heading integration) runs and is
  tested on a host PC with a stubbed Arduino API, no hardware required.
- Compile CI. GitHub Actions compiles the sketch on every push and pull request
  with both arduino-cli and PlatformIO, using the ATmega32U4-compatible Leonardo
  target.
- A PlatformIO project alongside the plain Arduino sketch, so the firmware builds
  through the Arduino IDE, arduino-cli, or PlatformIO without moving any files.
- Statistical re-analysis. The figures and headline numbers are re-derived from
  the raw trial CSVs in `data/` by `analysis/make_gallery.py` and
  `analysis/sensor_noise.py`, rather than quoted from the original write-up. The
  scripts defend against the PuTTY headers, label rows, and delimiter-free theta
  dumps in the old logs. This re-analysis is what produced the mixed result
  reported in the README: a clear win over raw analog, a tie or slight loss
  against the already-steady digital array.
- Formal statistics. `analysis/statistics.py` puts 95 percent bootstrap
  confidence intervals on those numbers and runs pairwise significance tests
  (see `docs/statistics.md`). The switching versus analog reduction on curves is
  significant (p on the order of 1e-8); switching versus digital only ties on
  straights and is marginally worse on curves.
- Offline replay harness. `sim/` runs all four strategies on one identical,
  seeded set of synthetic sensor inputs and prints smoothness and tracking
  metrics. This is the apples-to-apples comparison the physical runs could not
  give, because each real run had a different trajectory and sample count. On the
  synthetic track the fusion mode gives the smoothest steering while tracking
  nearly as tightly as digital, and the debounce holds the switch to a modest
  rate.

## What still requires the physical robot

The changes above are implemented and build, but several things cannot be settled
without running the actual robot on a track.

- On-track tuning of the PID gains, the switch hysteresis margin, and the dwell
  time. Sensible defaults are set, but the right values depend on the specific
  robot and surface.
- Fresh runs with the clean timestamped CSV logger. The existing dataset has
  short, uneven heading logs (switching has about three times fewer samples than
  analog or digital), so the smoothness comparison is directional only. A
  statistically solid smoothness comparison needs more and longer runs,
  especially for the switching and fusion modes, logged in the new clean format.
