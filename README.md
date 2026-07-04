<div align="center">

# Bifocal

**A line-following robot that reads its sensors two ways, analog and digital, and every control cycle steers with whichever reading is more stable.**

![Platform: Arduino](https://img.shields.io/badge/platform-Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![Board: Pololu 3Pi+ 32U4](https://img.shields.io/badge/board-Pololu%203Pi%2B%2032U4-2A6DB0?style=flat-square)
![Language: C++](https://img.shields.io/badge/firmware-C%2B%2B-00599C?style=flat-square&logo=cplusplus&logoColor=white)
![Analysis: Jupyter](https://img.shields.io/badge/analysis-Jupyter-F37626?style=flat-square&logo=jupyter&logoColor=white)
![Paper: PDF](https://img.shields.io/badge/paper-PDF-B31B1B?style=flat-square)

<img src="gallery/variance_comparison.png" width="460">

</div>

Bifocal is the firmware and experimental study for a **Pololu 3Pi+ 32U4**
line-following robot. It reads the same 5-element infrared sensor array two ways.
One is a high-resolution **analog** read; the other is a robust **digital**
(RC decay-time) read. On every control cycle the robot measures how consistent
each read is, using variance, and steers with the array that is currently
steadier. It needs no extra hardware, only the sensors already on the board.

The figure above is the main result, computed from the robot's own trial logs.
Switching cuts the analog array's worst-case noise (its variance on curves drops
by about 75 percent), but the switching itself adds some heading wobble. That is
the central trade-off this repository documents: steadier readings, slightly less
smooth motion.

## Gallery

Every figure is generated from the raw trial CSVs in `data/` by
[`analysis/make_gallery.py`](analysis/make_gallery.py). No values are entered by
hand.

<table>
<tr>
<td align="center"><img src="gallery/variance_comparison.png" width="300"><br><sub><b>Variance by method</b>: switching removes analog's curve spike and ties digital</sub></td>
<td align="center"><img src="gallery/theta_stability.png" width="300"><br><sub><b>Heading spread</b>: analog holds the straightest line; switching wobbles (preliminary)</sub></td>
<td align="center"><img src="gallery/calibration_contrast.png" width="300"><br><sub><b>Calibration contrast</b>: the line reads as a peak on sensor 2; digital's floor is tighter</sub></td>
</tr>
</table>

## About

Bifocal started as an undergraduate robotics assignment, getting a Pololu 3Pi+ to
follow a black line on a white floor, and grew into a small study around one
question. If a robot can sense the same line two different ways, can it decide by
itself, moment to moment, which way to trust?

A reflectance sensor can be read in two modes, and the two behave differently.
An **analog** read gives a smooth, high-resolution value that is precise but
noisy, with occasional large spikes. A **digital** read charges the sensor's
capacitor and times how long it takes to discharge, which gives a cleaner, almost
thresholded value that is robust but coarser. Most robots pick one mode and keep
it. Bifocal runs both at once and chooses between them continuously.

The rule is simple. Each cycle the robot samples both arrays, computes the
average per-sensor variance of each, and steers using the array with the lower
variance. The idea is to follow the sensor that is behaving well right now rather
than the one that is better on paper. The whole mechanism is software and costs
only a few milliseconds of sampling per cycle.

To test whether this helps, the robot was run as a controlled experiment. Three
strategies, analog only, digital only, and variance-based switching, were each
driven over straight and curved track segments while the firmware logged
per-sensor variance and encoder-derived heading. Re-analysing those logs gives a
mixed result. Switching is a clear improvement over raw analog: median sensor
variance falls by about 28 percent on straights and about 75 percent on curves,
and it removes analog's roughly tenfold worst-case spikes. Against the digital
array, which is already steady, switching only ties or slightly loses. On the
thin heading data, the extra switching makes motion visibly less smooth than
analog alone. Readings get steadier, motion gets a little rougher.

This repository holds the whole project: the refactored firmware that runs the
three strategies, the raw trial data, the analysis that turns it into the figures
above, and the [technical paper](docs/variance-based-sensor-switching-paper.pdf)
that wrote it up. The paper is by nz20469 and xi20942, and the encoder and
odometry scaffolding follows University of Bristol course material by Paul O'Dowd.

## How it works

The robot reads a 5-element downward-facing IR reflectance array
(index `0` = left, `2` = middle, `4` = right) in two independent ways.

- **Analog** (`analoglinesensors.h`): `analogRead()` of each phototransistor,
  averaged over several samples. Higher resolution, more noise.
- **Digital** (`digitallinesensors.h`): charge the sensor capacitor, then time
  its discharge. Longer time means a darker surface. Cleaner contrast, and
  effectively a robust thresholded read.

Both are normalised to the range 0 to 1 per sensor by a spin-in-place
**calibration** that captures each sensor's minimum (white) and maximum (black).

**The switch.** Each cycle the firmware computes the average per-sensor variance
of each array over a short window and picks the steadier one.

```text
if (analogVariance < digitalVariance)  use analog
else                                   use digital
```

**Steering.** A weighted term from the mid-left (`1`) and mid-right (`3`) sensors
drives a differential around a forward bias.

```text
W        = calibrated[3]/(calibrated[1]+calibrated[3]) - calibrated[1]/(...)
LeftPWM  = BiasPWM + MaxTurnPWM * W
RightPWM = BiasPWM - MaxTurnPWM * W
```

Wheel-encoder **odometry** (`kinematics.h`) integrates heading (`theta`) so every
run can be logged and analysed offline.

## Features

- **Three interchangeable strategies**, analog only, digital only, and
  variance-based switching, selected with one `#define` (`FOLLOW_METHOD`).
- **Dual sensor pipelines** sharing one physical array, analog `analogRead` and
  digital RC-decay timing, each with its own calibration and variance.
- **Variance-based arbitration** using per-sensor variance over a sample window.
- **Spin-in-place auto-calibration** that normalises every sensor to 0 to 1.
- **Differential-drive odometry** that logs heading over each trial.
- **A reproducible analysis pipeline** (`analysis/`) that regenerates every
  figure in this README from the raw CSVs.
- **A full experimental dataset**, calibration sweeps and straight and curved
  trials for all three strategies.

## Tech stack

| | Tool | Role |
|-|------|------|
| <img src="assets/logos/arduino.svg" height="20"> | Arduino (AVR toolchain) | Build and flash the ATmega32U4 firmware |
| <img src="assets/logos/cplusplus.svg" height="20"> | C++ | Firmware, written as an Arduino sketch plus headers |
| <img src="assets/logos/python.svg" height="20"> | Python | Data analysis and figure generation |
| <img src="assets/logos/jupyter.svg" height="20"> | Jupyter | Exploratory notebook (`analysis/RT_Results.ipynb`) |
| <img src="assets/logos/pandas.svg" height="20"> | pandas | Reading and wrangling the trial CSVs |
| <img src="assets/logos/numpy.svg" height="20"> | NumPy | Variance, statistics, array maths |
| <img src="assets/logos/matplotlib.svg" height="20"> | matplotlib / seaborn | Plotting the gallery figures |

## Hardware

| Component | Detail |
|-----------|--------|
| Controller | Pololu **3Pi+ 32U4** (ATmega32U4) or a Romi/32U4 control board |
| Line sensors | Onboard 5-element IR reflectance array |
| Drive | Two micro-metal gearmotors with quadrature encoders |
| Track | Black line on a white surface (invert calibration to reverse) |

Pin assignments are at the top of each header (`A_LS_*`, `D_LS_*`, `L_PWM`,
encoder pins, and so on). Adjust them if your wiring differs.

## Building

The firmware is a standard Arduino sketch. The sketch folder name must match the
`.ino` file name, so keep them together as `firmware/line_following/`.

### Arduino IDE

1. Install board support for the **Pololu A-Star 32U4 / Arduino Leonardo**
   (ATmega32U4) core.
2. Open `firmware/line_following/line_following.ino`.
3. Select the board and port, then **Upload**.

### arduino-cli

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following
arduino-cli upload  --fqbn arduino:avr:leonardo -p COM3 firmware/line_following
```

On boot the robot prints `***RESET***`, spins to calibrate, beeps about ten times
(move it to the start line during this window), then runs a trial and logs
heading over serial at 9600 baud.

## Configuration

Edit the `#define`s at the top of `line_following.ino`.

| Macro | Purpose | Default |
|-------|---------|---------|
| `FOLLOW_METHOD` | `METHOD_ANALOG`, `METHOD_DIGITAL`, or `METHOD_SWITCHING` | `METHOD_SWITCHING` |
| `DEBUG_INSPECT` | `1` streams calibrated readings and variance forever (tuning) | `0` |
| `BiasPWM` | forward bias PWM | `30` |
| `MaxTurnPWM` | maximum steering differential | `20` |
| `MAX_RESULTS` | heading samples logged per trial | `80` |
| `ANALOG_STOP_SUM`, `DIGITAL_STOP_SUM` | end-of-line thresholds | `0.4`, `0.1` |

Switching between the three strategies is a one-line change. This is how the
three datasets in `data/results/` were produced.

## Experimental method

Each of the three strategies was run over two track types, straight and curve.
Per run the firmware recorded per-sensor **variance** over time and
encoder-odometry **heading** (`theta`). Procedure, from the paper: charge the
battery, place the middle sensor over the line, run the spin calibration, realign
to the start, then record until the end of the line is detected.

## Results

Numbers are re-derived from the raw CSVs in `data/results/`. Medians are quoted
where the distributions are dominated by rare spikes (see
[limitations](#known-limitations)).

**Sensor-reading variance** (lower is steadier):

| Comparison | Straight (median) | Curve (median) | Verdict |
|------------|-------------------|----------------|---------|
| Switching vs Analog | down 28% | down 75% | clear win |
| Switching vs Digital | down 25% | up 29% | ties, slightly worse on curves |

- The real benefit of switching is removing analog's worst case. Analog's mean
  curve variance (about 0.0057) is roughly ten times switching's (about 0.00054).
- Against digital, switching only ties or slightly loses, because digital is
  already the low-variance baseline. The value of switching is avoiding analog's
  spikes.
- Noisiest sensor: analog at the edges, digital at index 1, switching at the
  middle sensor (index 2). Edge sensors are the quietest.

**Calibration contrast** (middle sensor, black to white): analog goes 0.99 to
0.22 (a gap of about 0.77), digital goes 0.99 to 0.08 (a gap of about 0.91), so
digital separates the line from the background better.

**Heading smoothness** (straight line, preliminary): analog held the straightest
line (std about 0.062 rad), digital was in the middle (about 0.078), and
switching wobbled most (about 0.205). Treat this as directional only, because the
switching run has about three times fewer samples than analog or digital.

**Takeaway.** The variance-based switch improves sensor reliability, but the
switching itself adds navigation instability. This matches the paper's
conclusion: an accuracy-versus-smoothness trade-off, not a one-sided win.

## Reproducing the analysis

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install pandas numpy matplotlib seaborn jupyter

python analysis/make_gallery.py        # regenerate the gallery/ figures
jupyter notebook analysis/RT_Results.ipynb
```

`RT_Results.ipynb` loads its combined `theta.csv` and `variances.csv` from a
remote URL, so it runs without local paths. `make_gallery.py` reads the
per-scenario CSVs in `data/` directly and defends against the PuTTY headers,
label rows, and delimiter-free theta dumps in the raw logs.

## Project structure

```text
bifocal/
├── firmware/
│   └── line_following/           # Arduino sketch (open this folder in the IDE)
│       ├── line_following.ino     #   setup(), loop(), strategy dispatch
│       ├── analoglinesensors.h    #   AnalogLineSensors_c  (analogRead pipeline)
│       ├── digitallinesensors.h   #   DigitalLineSensors_c (RC decay pipeline)
│       ├── motors.h               #   Motors_c (PWM and direction)
│       ├── encoders.h             #   quadrature encoder ISRs (AVR register-level)
│       └── kinematics.h           #   Kinematics_c (differential-drive odometry)
├── data/
│   ├── calibration/              # 8 CSVs: per-sensor readings and variance,
│   │                             #   analog/digital by black-line/white surface
│   └── results/                  # 11 CSVs: theta and per-sensor variance for
│                                 #   analog/digital/switching on straight and curve
├── analysis/
│   ├── RT_Results.ipynb          # exploratory notebook (pandas, seaborn)
│   └── make_gallery.py           # regenerates the README figures from data/
├── gallery/                      # generated result figures (checked in)
├── docs/
│   └── variance-based-sensor-switching-paper.pdf   # the technical paper
└── README.md
```

Every data CSV is distinct (verified by checksum), so none were removed. Only a
duplicated copy of the firmware and its zip were cleaned up.

## Known limitations

1. **Thin heading data.** Theta logs are short and uneven, and switching has
   about three times fewer samples, so the smoothness numbers are indicative
   rather than statistically robust.
2. **Accumulating serial dumps.** The theta logger reprints its whole history on
   each line with no delimiters, so the analysis takes the longest run per file.
3. **PuTTY log headers** are embedded in several CSVs and must be stripped.
4. **The `-1.00` sentinel** appears in the switching theta log. Confirm its
   meaning against the firmware before treating those rows as data.
5. **Outlier-dominated means.** Variance means are skewed by rare spikes, so the
   conclusions lean on medians.
6. **Two-sensor steering.** Steering uses only sensors 1 and 3.

## Roadmap

**Controller.** Use a full 5-sensor weighted position (or a PID), debounce the
switch with hysteresis or a minimum dwell time (the likely source of the wobble),
or fuse the two arrays by inverse-variance weight instead of a hard pick.

**Sensing.** Add a low-pass filter on calibrated readings, and investigate why
the middle sensor is noisiest under switching.

**Odometry.** Calibrate `wheelRadius`, `wheelSeparation`, and
`encoderCountsPerRevolution` against ground truth. The heading formula was
corrected in this refactor.

**Data.** Log with timestamps and a fixed sample rate, emit clean CSV, and
collect more and longer runs, especially for switching.

**Tooling.** Add a PlatformIO project or a CI `arduino-cli compile` check, and
host-side unit tests for the pure logic. A stubbed Arduino API already
syntax-verifies the headers under `g++ -Wall -Wextra`.

## Firmware fixes in this refactor

The core algorithm's behaviour is preserved. These changes remove real bugs and
undefined behaviour. In short: added the missing `motors.initialise()`, removed a
stray brace and an always-on `while(true)` block that stranded `loop()`, made
`calculateVariance()` actually return, replaced variable-length arrays with fixed
ones, fixed the analog sampling loop that discarded all but the last sample, made
analog and digital average-variance consistent so the switch compares like with
like, de-collided the two headers' pin macros (`A_` and `D_` prefixes), fixed a
`thetaLog` buffer overflow, and corrected the differential-drive heading update to
`deltaTheta = (dR - dL) / wheelSeparation` (previously divided by twice the
separation). Re-baseline before comparing new theta values to old runs.

## Credits and licence

The full write-up is in
[`docs/variance-based-sensor-switching-paper.pdf`](docs/variance-based-sensor-switching-paper.pdf):
*Investigating Line-Following Robot Navigation through Variance-Based Sensor
Switching: A Hybrid Approach to Optimizing Sensor Performance*, by nz20469 and
xi20942. Encoder and odometry scaffolding and inline guidance credit Paul O'Dowd
(University of Bristol course material).

No licence file is included yet. Add one (for example MIT) if you intend others to
reuse this code.
