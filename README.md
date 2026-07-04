# Variance-Based Sensor Switching for a Line-Following Robot

A hybrid line-following controller for a Pololu **3Pi+ / Romi**-class robot that
dynamically switches between an **analog** and a **digital** IR sensor array
based on real-time reading **variance** — using whichever array is currently
more stable — with no extra hardware.

This repository contains the robot firmware, the raw experimental data from
straight- and curved-track trials, the analysis notebook, and the accompanying
technical paper.

> **In one line:** switching cuts sensor-reading variance ~75% (median) below raw
> analog on curves, but the extra switching introduces heading instability — so
> it trades a small amount of navigation smoothness for a large gain in sensor
> reliability. See [Results](#results).

---

## Contents

- [How it works](#how-it-works)
- [Repository structure](#repository-structure)
- [Hardware](#hardware)
- [Build &amp; flash](#build--flash)
- [Configuration](#configuration)
- [Experimental method](#experimental-method)
- [Results](#results)
- [Reproducing the analysis](#reproducing-the-analysis)
- [Known limitations](#known-limitations)
- [Build follow-ups / roadmap](#build-follow-ups--roadmap)
- [What changed in this refactor](#what-changed-in-this-refactor)
- [Paper, authors &amp; license](#paper-authors--license)

---

## How it works

The robot reads a 5-element downward-facing IR reflectance array (index `0` =
left … `2` = middle … `4` = right) in two different ways:

- **Analog** (`analoglinesensors.h`) — `analogRead()` of each phototransistor,
  averaged over several samples. High resolution, but more susceptible to noise.
- **Digital** (`digitallinesensors.h`) — the classic RC decay-time method: charge
  the sensor capacitor, then time how long it takes to discharge. Cleaner
  black/white contrast, effectively a robust thresholded reading.

Both readings are normalised to `0..1` per sensor during a spin-in-place
**calibration** that captures each sensor's min (white) and max (black).

### The switching mechanism

Every control tick the firmware computes the **average per-sensor variance** of
each array over a short sample window (lower variance = steadier signal) and
selects the array with the lower variance to drive steering:

```text
if (analogVariance < digitalVariance)  use analog
else                                   use digital
```

### Steering

Steering uses a weighted measurement from the mid-left (`1`) and mid-right (`3`)
sensors:

```text
W = calibrated[3] / (calibrated[1] + calibrated[3])   // right weight
  - calibrated[1] / (calibrated[1] + calibrated[3])   // left  weight
```

`W ∈ [-1, 1]` is applied as a differential around a forward bias:

```text
LeftPWM  = BiasPWM + MaxTurnPWM * W
RightPWM = BiasPWM - MaxTurnPWM * W
```

Wheel-encoder **odometry** (`kinematics.h`) integrates heading (`theta`) so runs
can be logged and analysed offline.

---

## Repository structure

```text
.
├── firmware/
│   └── line_following/           # Arduino sketch (open this folder in the IDE)
│       ├── line_following.ino     # setup(), loop(), strategy dispatch
│       ├── analoglinesensors.h    # AnalogLineSensors_c
│       ├── digitallinesensors.h   # DigitalLineSensors_c
│       ├── motors.h               # Motors_c (PWM + direction)
│       ├── encoders.h             # quadrature encoder ISRs (AVR register-level)
│       └── kinematics.h           # Kinematics_c (differential-drive odometry)
├── data/
│   ├── calibration/              # 8 CSVs: per-sensor readings & variance,
│   │                             #   analog/digital × black-line/white surface
│   └── results/                  # 11 CSVs: theta-over-time and per-sensor
│                                 #   variance for analog/digital/switching on
│                                 #   straight & curved tracks
├── analysis/
│   └── RT_Results.ipynb          # pandas/seaborn analysis + plots
├── docs/
│   └── line-following.pdf        # the technical paper
└── README.md
```

Every data CSV is distinct (verified by checksum) — none were removed. Only the
old duplicated `Code/` copy of the firmware and the redundant `Code.zip` were
removed during cleanup.

---

## Hardware

- Pololu **3Pi+ 32U4** (ATmega32U4) or a Romi/32U4 control board
- Onboard 5-element IR reflectance sensor array
- Two micro-metal gearmotors with quadrature encoders
- A line track: **black line on a white surface** (or invert calibration)

Pin assignments live at the top of each header (`*_LS_*_PIN`, `L_PWM`, encoder
pins, etc.). Adjust them if your wiring differs.

---

## Build & flash

The firmware is a standard Arduino sketch. The sketch folder name must match the
`.ino` file name — keep them together as `firmware/line_following/`.

### Arduino IDE

1. Install board support for the **Pololu A-Star 32U4 / Arduino Leonardo**
   (ATmega32U4) core.
2. Open `firmware/line_following/line_following.ino`.
3. Select the correct board and port, then **Upload**.

### arduino-cli

```bash
# one-time: install the AVR core
arduino-cli core install arduino:avr

# compile (Leonardo is 32U4-compatible; use your Pololu FQBN if installed)
arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following

# upload (replace the port)
arduino-cli upload  --fqbn arduino:avr:leonardo -p COM3 firmware/line_following
```

On boot the robot: prints `***RESET***`, spins in place to calibrate, beeps ~10
times (move it to the start line during this window), then runs a trial and logs
heading over serial at 9600 baud.

---

## Configuration

Edit the `#define`s at the top of `line_following.ino`:

| Macro | Purpose | Default |
|---|---|---|
| `FOLLOW_METHOD` | `METHOD_ANALOG`, `METHOD_DIGITAL`, or `METHOD_SWITCHING` | `METHOD_SWITCHING` |
| `DEBUG_INSPECT` | `1` = stream calibrated readings/variance forever (tuning) | `0` |
| `BiasPWM` | forward bias PWM | `30` |
| `MaxTurnPWM` | max steering differential | `20` |
| `MAX_RESULTS` | number of heading samples logged per trial | `80` |
| `ANALOG_STOP_SUM` / `DIGITAL_STOP_SUM` | end-of-line detection thresholds | `0.4` / `0.1` |

Switching between the three strategies is a one-line change to `FOLLOW_METHOD` —
this is how the three data sets in `data/results/` were produced.

---

## Experimental method

Three strategies (**analog**, **digital**, **switching**) were each run on two
track types (**straight** and **curve**). Per run the firmware recorded:

- **theta** — heading over time (encoder odometry) → `*_straight_line_theta.csv`
- **per-sensor variance** over time → `*_variances_{straight_line,curve}.csv`

Procedure (from the paper): fully charge the battery, place the robot with the
middle sensor over the line, run the spin calibration, realign to the start, then
record theta and variance until the end of the line is detected.

---

## Results

Numbers below come from re-analysing the raw CSVs in `data/results/`
(medians are quoted where distributions are spike-heavy; see caveats).

### Sensor-reading variance (lower = steadier)

| Comparison | Straight (median) | Curve (median) |
|---|---|---|
| **Switching vs Analog** | **−28%** | **−75%** ✅ big win |
| **Switching vs Digital** | −25% (small win) | **+29%** ❌ slightly worse |

- Switching's headline benefit is **taming analog's worst case**: analog's mean
  variance on curves (~0.0057) is roughly **10×** switching's (~0.00054).
- Against digital, switching only **ties or slightly loses** — digital is already
  the low-variance baseline. Switching's value is *avoiding analog's spikes*, not
  beating digital outright.
- **Noisiest sensor:** analog spikes at edge/near-edge sensors; digital at
  index 1; switching at the **middle sensor (index 2)**. Edge sensors are
  quietest.

### Calibration contrast (black vs white, middle sensor)

| Array | Black | White | Separation |
|---|---|---|---|
| Analog | 0.99 | 0.22 | Δ ≈ 0.77 |
| Digital | 0.99 | 0.08 | Δ ≈ **0.91** |

Digital gives the cleaner black/white separation and a lower off-line baseline.

### Heading smoothness (straight line, **preliminary**)

On the (small, single-run) heading logs, **analog held the straightest line**
(std ≈ 0.062, mean step ≈ 0.034 rad), digital was middling, and **switching
wobbled most** (std ≈ 0.205, mean step ≈ 0.192 rad).

⚠️ Treat this as **directional only** — the theta captures are short and uneven,
and the switching run has ~3× fewer samples than analog/digital (see below).

### Takeaway

This matches the paper's conclusion: the variance-based switch **improves
sensor-reading reliability** (large variance reduction vs analog) but the act of
switching **introduces navigation instability**, making motion less smooth than
analog alone. It is a genuine accuracy-vs-smoothness trade-off, not a strict win.

---

## Reproducing the analysis

`analysis/RT_Results.ipynb` loads its combined `theta.csv` / `variances.csv` from
a remote URL (`raw.githubusercontent.com/paulodowd/GeneralPurpose/R-T/`), so it
runs without local paths. To run locally against the per-scenario CSVs in
`data/results/` instead, point `pd.read_csv(...)` at those files.

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install pandas numpy matplotlib seaborn jupyter
jupyter notebook analysis/RT_Results.ipynb
```

---

## Known limitations

These are data-quality and design caveats surfaced while analysing the results:

1. **Thin heading data** — theta logs are short and uneven (switching has ~3×
   fewer samples than analog/digital); heading-smoothness numbers are indicative,
   not statistically robust.
2. **Accumulating serial dumps** — the theta logger reprints its whole history
   each line (triangular growth), which inflates naive row counts. The analysis
   uses the longest continuous run per file.
3. **PuTTY log headers** are embedded in several CSVs and must be stripped before
   parsing.
4. **`-1.00` sentinel** appears in the switching theta log — confirm its meaning
   against the firmware before treating those rows as data.
5. **Outlier-dominated means** — variance means are skewed by rare spikes (analog
   straight-line max variance hits 1.0 vs a 3e-5 median); conclusions lean on
   medians.
6. **Two-sensor steering** — steering uses only sensors 1 and 3, discarding the
   middle and edge sensors' information.

---

## Build follow-ups / roadmap

Concrete next steps, roughly in priority order:

**Controller**
- [ ] Replace the 2-sensor weighted term with a **full 5-sensor weighted
      position** (or a PID on line error) for smoother tracking.
- [ ] **Debounce the switch**: add hysteresis / a minimum dwell time so the array
      selection can't flip every tick — the likely source of the observed
      switching wobble.
- [ ] Fuse rather than switch: blend analog and digital by an inverse-variance
      weight instead of a hard pick.

**Sensing**
- [ ] Add a light **low-pass / moving-average filter** on calibrated readings.
- [ ] Investigate why the **middle sensor (index 2)** is noisiest under switching.

**Odometry**
- [ ] Calibrate `wheelRadius`, `wheelSeparation`, and `encoderCountsPerRevolution`
      in `kinematics.h` against measured ground truth (currently nominal values).
      Note the heading-update formula was corrected in this refactor (see below).

**Instrumentation / data**
- [ ] Log with **timestamps** and a fixed sample rate so cross-method comparisons
      are apples-to-apples.
- [ ] Emit **clean CSV** (no reprinted history, no PuTTY headers, documented
      sentinels) to remove the parsing gymnastics.
- [ ] Collect **more and longer runs** — especially for switching — for
      statistically meaningful heading-smoothness comparisons.

**Tooling**
- [ ] Add a PlatformIO project / CI `arduino-cli compile` check.
- [ ] Consider unit-testing the pure logic (variance, weighting, kinematics) on
      host with a mocked Arduino API (a stub was used to syntax-verify this
      refactor).

---

## What changed in this refactor

The firmware was reorganised and a number of real bugs were fixed. Behaviour of
the core algorithm is preserved; the fixes remove undefined behaviour and
compile/logic errors.

- **`main.ino` → `line_following.ino`**
  - Removed a **stray closing brace** and the empty `#define SENSOR_THRESHOLD`
    that broke the build.
  - Added the missing **`motors.initialise()`** call (motor pins were never set
    to `OUTPUT`).
  - Removed the always-on **`while(true)` debug block** that stranded `loop()`
    (the trial code was unreachable); it is now behind `DEBUG_INSPECT`.
  - Restored the results-printing state and added a clean strategy dispatch
    (`FOLLOW_METHOD`).
- **Sensor headers**
  - `calculateVariance()` declared `float` but **returned nothing** (undefined
    behaviour) → now `void`, storing into `variance[]`.
  - Replaced the **variable-length arrays** (`float samples[5][num_samples]`)
    with fixed-size arrays.
  - Fixed the analog `readAllSensors()` sampling loop that **discarded all but
    the last** sample; it now averages.
  - Made analog/digital **average variance consistent** (both mean over all 5
    sensors) so the switch compares like with like; removed `Serial`/`delay`
    calls from inside the selection path.
  - Guarded calibration against **divide-by-zero**.
  - **De-collided macros** (`A_`/`D_` prefixes) — both headers previously
    `#define`d the same pin macro names with different values.
- **`kinematics.h`**
  - Fixed a **buffer overflow**: `thetaLog[50]` was indexed up to 100.
  - Corrected the heading update to the standard differential-drive form
    `Δθ = (dRight − dLeft) / wheelSeparation` (was dividing by `2 ×`
    separation, under-reporting rotation 2×). *This changes logged theta scale —
    recalibrate/re-baseline before comparing to old runs.*
- **`motors.h`** — clamp via `constrain()`, removed dead post-`abs()` checks.
- **Structure** — split into `firmware/ data/ analysis/ docs/`, removed the
  duplicated `Code/` folder and `Code.zip`, added `.gitignore`.

The refactored headers were syntax-verified off-target with a stubbed Arduino API
(clean compile under `g++ -Wall -Wextra`).

---

## Paper, authors & license

The full write-up is in [`docs/line-following.pdf`](docs/line-following.pdf):
*"Investigating Line-Following Robot Navigation through Variance-Based Sensor
Switching: A Hybrid Approach to Optimizing Sensor Performance."*

Authors: **nz20469**, **xi20942**. Encoder/odometry scaffolding and inline
guidance credited to Paul O'Dowd (course material).

No license file is currently included — add one (e.g. MIT) if you intend others
to reuse this code.
