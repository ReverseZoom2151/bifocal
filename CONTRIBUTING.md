# Contributing to Bifocal

Bifocal is a line-following robot project for the Pololu 3Pi+ 32U4. It is built by
the authors listed in [AUTHORS](AUTHORS): Tiberiu Toca and Ronaldo Balram, with
encoder and odometry scaffolding following University of Bristol course material
by Paul O'Dowd. This guide explains how to build, test, and contribute changes.

## Repository layout

```text
firmware/   Arduino firmware. The main sketch is firmware/line_following/, with
            its headers in the same folder. firmware/tools/ holds standalone
            helper sketches (for example odometry_calibration).
data/       Raw trial CSVs. data/calibration/ and data/results/ hold per-sensor
            readings, variance, and heading logs for every method and track.
analysis/   Python scripts and a notebook that turn the raw CSVs into figures and
            statistics.
tests/      Host unit tests. They stub the Arduino API and run under g++, so no
            hardware is needed.
sim/        Offline replay harness. Runs all four strategies on identical
            synthetic inputs and reports smoothness and tracking metrics, on the
            host with no hardware.
docs/       Written documentation, including the technical paper, the build and
            CI notes, the odometry calibration procedure, the sensor-noise
            investigation, the statistical analysis, and the paper addendum.
gallery/    Generated figures, checked in.
```

## Building the firmware

The firmware is a standard Arduino sketch. The sketch folder name must match the
`.ino` file name, so keep them together as `firmware/line_following/`. The target
is an ATmega32U4; the Arduino Leonardo definition is used as a compile-compatible
stand-in. See [docs/build-and-ci.md](docs/build-and-ci.md) for full detail and
caveats.

### Arduino IDE

1. Install board support for the Pololu A-Star 32U4 / Arduino Leonardo
   (ATmega32U4) core.
2. Open `firmware/line_following/line_following.ino`.
3. Select the board and port, then Upload.

### arduino-cli

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following
arduino-cli upload  --fqbn arduino:avr:leonardo -p COM3 firmware/line_following
```

### PlatformIO

```bash
pio run                 # build the default environment
pio run -e leonardo     # build the leonardo (CI) environment
pio run --target upload # build and flash
```

## Running the host tests

The pure logic runs and is tested on the host PC, no hardware required.

```bash
cd tests && make test
```

This builds and runs all suites under `g++ -Wall`. The tests stub the Arduino API
and check the variance computations and the differential-drive pose and heading
integration. Add or update tests when you change that logic.

## Running the replay simulation

The offline harness compares all four strategies on identical synthetic inputs,
on the host with no hardware:

```bash
cd sim && make run
```

It prints a table of smoothness and tracking metrics (steering RMS, integral of
absolute error, switch rate). The inputs are seeded and deterministic.

## Regenerating figures and statistics

The figures in `gallery/` and the headline numbers are re-derived from the raw
CSVs in `data/`. No values are entered by hand. Set up the Python environment
once:

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install pandas numpy matplotlib seaborn jupyter
```

Then regenerate:

```bash
python analysis/make_gallery.py    # main gallery figures
python analysis/sensor_noise.py    # middle-sensor noise figure and stats
python analysis/statistics.py      # bootstrap CIs and significance tests
```

The exploratory notebook `analysis/RT_Results.ipynb` is also available. If you
change a figure, regenerate it with the script and commit the updated image so the
checked-in gallery stays in sync.

## Coding conventions

The firmware runs on an AVR microcontroller with very limited RAM, so the C++ is
deliberately conservative.

- No dynamic allocation. Do not use `new`, `malloc`, or anything that allocates on
  the heap.
- No STL and no exceptions.
- Fixed-size arrays only. No variable-length arrays.
- 2-space indentation.
- Plain ASCII only in source and documentation. No emojis, and no non-ASCII
  punctuation such as em-dashes or en-dashes.

Keep changes minimal and in the style of the surrounding code. New tunable
constants should be `#define`s near the top of the sketch, or PID gains in
`steering.h`, matching how existing configuration is handled.

## Continuous integration

CI must pass. GitHub Actions ([.github/workflows/ci.yml](.github/workflows/ci.yml))
compiles the sketch on every push and pull request with both arduino-cli and
PlatformIO. It is compile-only; nothing is flashed. Make sure the firmware still
compiles before opening a pull request, and run the host tests locally.

## Commits and pull requests

- Keep commits focused and give them clear, descriptive messages.
- Explain the intent of a change, not just the mechanics, especially for firmware
  behaviour.
- Make sure the firmware compiles and the host tests pass before opening a pull
  request.
- If a change affects a figure or a documented number, regenerate the figure and
  update the relevant document in `docs/` in the same pull request.
