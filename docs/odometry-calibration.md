# Odometry calibration

Bifocal tracks its pose by dead reckoning from the two wheel encoders. The
accuracy of that estimate depends on three physical constants in
`firmware/line_following/kinematics.h`:

- `wheelRadius` in mm (nominal 16.5)
- `wheelSeparation` in mm, the distance between the two wheel contact points
  (nominal 42.5)
- `encoderCountsPerRevolution`, the number of encoder counts per full wheel
  revolution (nominal 358.3)

From these the class derives `mmPerCount`, the linear distance one wheel travels
per encoder count:

    mmPerCount = (2 * PI * wheelRadius) / encoderCountsPerRevolution

The nominal numbers are close but not exact for any individual robot. Tyres wear,
gearboxes vary, and the effective wheel separation is not quite the mechanical
axle spacing. This document describes how to measure the constants against ground
truth and how to load the results into the firmware.

The helper sketch at `firmware/tools/odometry_calibration/odometry_calibration.ino`
automates the encoder-count collection. It drives the robot and prints the raw
counts and the derived ratios over serial at 9600 baud.

## What each test measures

There are two independent measurements.

1. A straight-line test fixes `mmPerCount` directly, and from it
   `encoderCountsPerRevolution` given a wheel radius.
2. A rotation test fixes `wheelSeparation`, using the `mmPerCount` from the first
   test.

Do the straight-line test first, because the rotation test depends on its result.

## Test 1: straight line (mmPerCount and counts per revolution)

Goal: find how many mm of travel one encoder count represents.

1. Mark a known distance on the floor, for example 500 mm. A longer distance
   averages out per-count error, so use as much space as you have.
2. Line the robot up at the start mark.
3. Zero both encoder counters, drive the robot straight to the finish mark, and
   record the encoder counts for each wheel. The helper sketch does the zeroing
   and printing for you; you only need to set the robot down and read the serial
   output.
4. Measure the actual distance the robot travelled with a ruler. Do not trust the
   commanded distance, because the open-loop drive time is approximate.

Compute the average count across the two wheels:

    averageCount = (leftCount + rightCount) / 2

Then:

    mmPerCount = measuredDistanceMm / averageCount

To back out counts per revolution, use your wheel radius (measure the wheel
diameter with calipers and halve it, or start from the nominal 16.5 mm):

    wheelCircumference = 2 * PI * wheelRadius
    encoderCountsPerRevolution = wheelCircumference / mmPerCount

You now have `mmPerCount`, `wheelRadius`, and `encoderCountsPerRevolution`. Note
that `mmPerCount` is the quantity that actually drives the odometry, so if your
measured wheel radius and counts-per-rev do not reproduce your measured
`mmPerCount` exactly, prefer keeping `mmPerCount` consistent (see the plugging-in
section below).

## Test 2: rotation (wheel separation)

Goal: find the effective distance between the wheel contact points.

1. Mark the robot's starting heading, for example with a strip of tape on the
   floor aligned to the chassis.
2. Zero both encoder counters and spin the robot in place through exactly one
   full turn (360 degrees) back to the marked heading. With the helper sketch,
   adjust `SPIN_TIME_MS` and repeat until the robot lands on a clean 360 degree
   turn.
3. Record the encoder counts. In a pure spin the wheels turn in opposite
   directions, so take the magnitude of each and average them:

    arcCount = (abs(leftCount) + abs(rightCount)) / 2

During a full in-place turn each wheel rolls along a circle whose diameter is the
wheel separation, so each wheel travels one turning circumference:

    arcDistance = arcCount * mmPerCount
    turningCircumference = PI * wheelSeparation

Setting the wheel arc distance equal to the turning circumference and solving:

    wheelSeparation = (arcDistance) / PI
                    = (arcCount * mmPerCount) / PI

Use the `mmPerCount` you found in Test 1. If you spun through N full turns
instead of one (spinning more turns reduces per-turn error), divide by N:

    wheelSeparation = (arcCount * mmPerCount) / (PI * N)

## Plugging the results into the firmware

`Kinematics_c` exposes a setter that overrides the defaults and recomputes
`mmPerCount`:

    void setGeometry(float wheelRadius, float wheelSeparation, float countsPerRev);

Call it once in `setup()` before the first `update()`, for example:

    kinematics.setGeometry(16.5, 42.5, 358.3);   // replace with your values

The setter recomputes `mmPerCount` from `wheelRadius` and `countsPerRev` using
the same formula shown above. This means the pair you pass must reproduce your
measured `mmPerCount`. If you measured `mmPerCount` directly and it does not match
your radius and counts-per-rev exactly, adjust `countsPerRev` so that the formula
lands on your measured `mmPerCount`:

    countsPerRev = (2 * PI * wheelRadius) / measuredMmPerCount

That keeps the linear odometry faithful to the measurement while leaving the
radius at whatever value you prefer to report.

To confirm which values the firmware is actually using, call `printGeometry()`
after `setGeometry()` and read the labelled row over serial. The individual
accessors `getWheelRadius()`, `getWheelSeparation()`, `getCountsPerRev()`, and
`getMmPerCount()` are also available.

## Expected pitfalls

- Wheel slip. If the wheels spin faster than the robot moves, the counts
  overstate the distance and every derived constant is wrong. Keep the drive PWM
  low, use a surface with grip, and start and stop gently. Slip is the single
  largest source of error.
- Encoder resolution. Each count is a finite step of travel, so over a short test
  distance the quantisation error is a large fraction of the result. Use a long
  straight-line distance and several full turns to average this down.
- Tyre wear and compression. The effective rolling radius changes as tyres wear
  and depends on the robot's weight pressing the tyre flat. Recalibrate after
  changing tyres or adding significant load, and calibrate with the robot at its
  normal running weight.
- Effective versus mechanical separation. The `wheelSeparation` that makes the
  odometry correct is usually a little different from the measured axle spacing,
  because the tyres contact the floor over a patch rather than a point. Trust the
  rotation-test value over a ruler measurement of the chassis.
- Uneven motors. If one wheel consistently travels further than the other for the
  same command, the robot curves during the straight-line test. Average the two
  wheel counts and keep the test distance long so a small curve has little effect.
- Surface consistency. Calibrate on the same surface you run on. A different floor
  changes grip and the effective rolling radius.
