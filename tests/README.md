# Host-side unit tests

These tests exercise the pure algorithmic logic of the Bifocal firmware on a
normal PC, with no Arduino hardware. The Arduino API is replaced by a small
stub (`arduino_stub.h`) so the firmware headers compile and run under g++.

## Requirements

- g++ with C++11 support
- GNU make

## Running

```
cd tests
make test
```

This builds and runs every test suite and prints a pass/fail summary. The
`make test` target exits non-zero if any check fails.

Other targets:

- `make build` compiles the test executables without running them
- `make clean` removes the built executables

## What is covered

- `test_kinematics.cpp` (`Kinematics_c::update`)
  - straight drive advances `xPos` and leaves `theta` near 0
  - equal-and-opposite wheel counts rotate `theta` by
    `(distRight - distLeft) / wheelSeparation`, with no net translation
  - zero counts (and repeated identical counts) do not change the pose

- `test_variance.cpp` (`AnalogLineSensors_c` variance path)
  - constant sensor input gives per-sensor variance near 0
  - alternating input gives positive variance matching the population variance
    formula (divide by N)
  - `calculateAverageVariance()` returns the mean of the 5 per-sensor variances

## How the sensor stub feeds synthetic values

`analogRead` and `digitalRead` are injectable. A test installs a function with
`setAnalogReadFn(fn)`; the stub calls it with the pin and a monotonically
increasing call index (reset with `resetAnalogRead()`). One `readAllSensors()`
call performs `A_MAX_SAMPLES * 5` analog reads, so holding the returned value
constant across each 50-call block makes every averaged reading exact while the
value is stepped between blocks. This makes the calibrated readings, and hence
the computed variance, fully deterministic and assertable.
