<div align="center">

# Bifocal

**A line-following robot that switches between analog and digital sight in real time — always looking through whichever lens is steadier.**

![Platform: Arduino](https://img.shields.io/badge/platform-Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![Board: Pololu 3Pi+ 32U4](https://img.shields.io/badge/board-Pololu%203Pi%2B%2032U4-2A6DB0?style=flat-square)
![Language: C++](https://img.shields.io/badge/firmware-C%2B%2B-00599C?style=flat-square&logo=cplusplus&logoColor=white)
![Analysis: Jupyter](https://img.shields.io/badge/analysis-Jupyter-F37626?style=flat-square&logo=jupyter&logoColor=white)
![Paper: PDF](https://img.shields.io/badge/paper-PDF-B31B1B?style=flat-square)

<img src="gallery/variance_comparison.png" width="460">

</div>

Bifocal is the firmware and experimental study for a **Pololu 3Pi+ 32U4**
line-following robot that carries two ways of reading the same infrared sensor
array — a high-resolution **analog** mode and a robust **digital** (RC decay-time)
mode — and, every control tick, hands steering to whichever one is currently
producing the *steadier* signal. It picks by **variance**: the array with the
lower reading variance wins. No extra hardware, no extra sensors — just a
smarter way to use the ones already on the board.

The result below is the headline finding, straight from the robot's own logs:
switching tames the analog array's worst-case noise (its curve variance drops by
~75%), but the act of switching adds some heading wobble. It is a real
**reliability-vs-smoothness trade-off**, and this repo has the data to show it.

## Gallery

All three figures are generated from the raw trial CSVs in `data/` by
[`analysis/make_gallery.py`](analysis/make_gallery.py) — no hand-drawn numbers.

<table>
<tr>
<td align="center"><img src="gallery/variance_comparison.png" width="300"><br><sub><b>Variance by method</b>: switching kills analog's curve spike; ties digital</sub></td>
<td align="center"><img src="gallery/theta_stability.png" width="300"><br><sub><b>Heading spread</b>: analog holds the straightest line (switching wobbles — preliminary)</sub></td>
<td align="center"><img src="gallery/calibration_contrast.png" width="300"><br><sub><b>Calibration contrast</b>: the line reads as a bell on sensor 2; digital's floor is tighter</sub></td>
</tr>
</table>

## About

Bifocal began as an undergraduate robotics assignment — get a Pololu 3Pi+ to
follow a black line on a white floor — and turned into a small research project
around a single question: **if a robot can sense the same line two different
ways, can it decide, on its own and moment to moment, which way to trust?**

A line sensor can be read two ways, each with a different personality. Read in
**analog**, the phototransistor gives a smooth, high-resolution value — precise,
but jittery, and prone to the occasional wild spike. Read in **digital** (the
classic charge-the-capacitor-and-time-the-discharge trick), it gives a clean,
almost thresholded reading — robust and well-behaved, but coarser. The usual
robot commits to one of these for life. Bifocal refuses to choose up front.

Instead it runs **both pipelines at once**, and every control tick it measures
how *consistent* each array's readings are — their variance — and steers using
the array that is currently steadier. The intuition: trust the sensor that is
behaving right now, not the one that is theoretically better. The whole
mechanism is software; it costs nothing but a few milliseconds of sampling on a
board you already have.

To find out whether it actually helps, the robot was run as a controlled
experiment. Three strategies — **analog-only**, **digital-only**, and
**variance-based switching** — were each driven over straight and curved track
segments while the firmware logged per-sensor variance and encoder-derived
heading. Re-analysing those logs gives an honest, mixed verdict: switching is a
**large, consistent win over raw analog** (median sensor variance down ~28% on
straights and ~75% on curves, taming analog's ~10× worst-case spikes), but it
only **ties the already-steady digital array**, and on the thin heading data the
extra switching makes motion visibly *less* smooth than analog alone. Reliability
up, smoothness down — a genuine engineering trade-off rather than a free lunch.

This repository is the whole story end to end: the refactored firmware that runs
the three strategies, the raw trial data, the analysis that turns it into the
figures above, and the [technical paper](docs/line-following.pdf) that wrote it
up. The accompanying paper is by **nz20469** and **xi20942**; the encoder and
odometry scaffolding follows University of Bristol course material by Paul O'Dowd.

## How it works

The robot reads a 5-element downward-facing IR reflectance array
(index `0` = left … `2` = middle … `4` = right) in two independent ways:

- **Analog** (`analoglinesensors.h`) — `analogRead()` of each phototransistor,
  averaged over several samples. High resolution, more noise.
- **Digital** (`digitallinesensors.h`) — charge the sensor capacitor, then time
  its discharge; longer time = darker surface. Cleaner contrast, effectively a
  robust thresholded reading.

Both are normalised to `0..1` per sensor by a spin-in-place **calibration** that
captures each sensor's min (white) and max (black).

**The switch.** Each tick the firmware computes the average per-sensor variance
of each array over a short window and picks the steadier one:

```text
if (analogVariance < digitalVariance)  use analog
else                                   use digital
```

**Steering.** A weighted term from the mid-left (`1`) and mid-right (`3`) sensors
drives a differential around a forward bias:

```text
W        = calibrated[3]/(calibrated[1]+calibrated[3]) - calibrated[1]/(...)
LeftPWM  = BiasPWM + MaxTurnPWM * W
RightPWM = BiasPWM - MaxTurnPWM * W
```

Wheel-encoder **odometry** (`kinematics.h`) integrates heading (`theta`) so every
run can be logged and analysed offline.

## Features

- **Three interchangeable strategies** — analog-only, digital-only, or
  variance-based switching — selected with one `#define` (`FOLLOW_METHOD`).
- **Dual sensor pipelines** sharing one physical array: analog `analogRead` and
  digital RC-decay timing, each with its own calibration and variance.
- **Variance-based arbitration** with per-sensor variance over a sample window.
- **Spin-in-place auto-calibration** normalising every sensor to `0..1`.
- **Differential-drive odometry** logging heading over each trial.
- **A reproducible analysis pipeline** (`analysis/`) that regenerates every
  figure in this README from the raw CSVs.
- **A full experimental dataset** — calibration sweeps and straight/curve trials
  for all three strategies.

## Hardware

| Component | Detail |
|-----------|--------|
| Controller | Pololu **3Pi+ 32U4** (ATmega32U4) or a Romi/32U4 control board |
| Line sensors | Onboard 5-element IR reflectance array |
| Drive | Two micro-metal gearmotors with quadrature encoders |
| Track | Black line on a white surface (invert calibration to reverse) |

Pin assignments live at the top of each header (`A_LS_*` / `D_LS_*`, `L_PWM`,
encoder pins, …). Adjust them if your wiring differs.

## Building

The firmware is a standard Arduino sketch. The sketch folder name must match the
`.ino` file name — keep them together as `firmware/line_following/`.

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

On boot the robot prints `***RESET***`, spins to calibrate, beeps ~10 times
(move it to the start line during this window), then runs a trial and logs
heading over serial at 9600 baud.

## Configuration

Edit the `#define`s at the top of `line_following.ino`:

| Macro | Purpose | Default |
|-------|---------|---------|
| `FOLLOW_METHOD` | `METHOD_ANALOG`, `METHOD_DIGITAL`, or `METHOD_SWITCHING` | `METHOD_SWITCHING` |
| `DEBUG_INSPECT` | `1` = stream calibrated readings/variance forever (tuning) | `0` |
| `BiasPWM` | forward bias PWM | `30` |
| `MaxTurnPWM` | max steering differential | `20` |
| `MAX_RESULTS` | heading samples logged per trial | `80` |
| `ANALOG_STOP_SUM` / `DIGITAL_STOP_SUM` | end-of-line thresholds | `0.4` / `0.1` |

Switching between the three strategies is a one-line change — this is exactly how
the three datasets in `data/results/` were produced.

## Experimental method

Each of the three strategies was run over two track types (**straight** and
**curve**). Per run the firmware recorded per-sensor **variance** over time and
encoder-odometry **heading** (`theta`). Procedure (from the paper): charge the
battery, place the middle sensor over the line, run the spin calibration, realign
to the start, then record until the end of the line is detected.

## Results

Numbers are re-derived from the raw CSVs in `data/results/`; medians are quoted
where distributions are spike-heavy (see [caveats](#known-limitations)).

**Sensor-reading variance** (lower = steadier):

| Comparison | Straight (median) | Curve (median) |
|------------|-------------------|----------------|
| Switching vs Analog | **−28%** | **−75%** ✅ |
| Switching vs Digital | −25% (small win) | **+29%** ❌ |

- Switching's real benefit is **taming analog's worst case**: analog's mean curve
  variance (~0.0057) is roughly **10×** switching's (~0.00054).
- Against digital it only **ties or slightly loses** — digital is already the
  low-variance baseline. Switching's value is *avoiding analog's spikes*.
- **Noisiest sensor:** analog at the edges, digital at index 1, switching at the
  **middle sensor (index 2)**; edge sensors are quietest.

**Calibration contrast** (middle sensor, black→white): analog `0.99 → 0.22`
(Δ ≈ 0.77); digital `0.99 → 0.08` (Δ ≈ **0.91**) — digital separates better.

**Heading smoothness** (straight line, **preliminary**): analog held the
straightest line (std ≈ 0.062 rad), digital middling (≈ 0.078), switching
wobbled most (≈ 0.205). Treat as directional only — the switching run has ~3×
fewer samples than analog/digital.

**Takeaway.** The variance-based switch **improves sensor reliability** but the
switching itself **adds navigation instability** — matching the paper's
conclusion. A real accuracy-vs-smoothness trade-off, not a strict win.

## Reproducing the analysis

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install pandas numpy matplotlib seaborn jupyter

python analysis/make_gallery.py        # regenerate the gallery/ figures
jupyter notebook analysis/RT_Results.ipynb
```

`RT_Results.ipynb` loads its combined `theta.csv` / `variances.csv` from a remote
URL, so it runs without local paths; `make_gallery.py` reads the per-scenario
CSVs in `data/` directly and defends against the PuTTY headers, label rows, and
delimiter-free theta dumps in the raw logs.

## Project structure

```text
bifocal/
├── firmware/
│   └── line_following/           # Arduino sketch (open this folder in the IDE)
│       ├── line_following.ino     #   setup(), loop(), strategy dispatch
│       ├── analoglinesensors.h    #   AnalogLineSensors_c  (analogRead pipeline)
│       ├── digitallinesensors.h   #   DigitalLineSensors_c (RC decay pipeline)
│       ├── motors.h               #   Motors_c (PWM + direction)
│       ├── encoders.h             #   quadrature encoder ISRs (AVR register-level)
│       └── kinematics.h           #   Kinematics_c (differential-drive odometry)
├── data/
│   ├── calibration/              # 8 CSVs: per-sensor readings & variance,
│   │                             #   analog/digital × black-line/white surface
│   └── results/                  # 11 CSVs: theta + per-sensor variance for
│                                 #   analog/digital/switching on straight & curve
├── analysis/
│   ├── RT_Results.ipynb          # exploratory notebook (pandas/seaborn)
│   └── make_gallery.py           # regenerates the README figures from data/
├── gallery/                      # generated result figures (checked in)
├── docs/
│   └── line-following.pdf        # the technical paper
└── README.md
```

Every data CSV is distinct (verified by checksum) — none were removed; only a
duplicated copy of the firmware and its zip were cleaned up.

## Known limitations

1. **Thin heading data** — theta logs are short and uneven (switching has ~3×
   fewer samples); smoothness numbers are indicative, not statistically robust.
2. **Accumulating serial dumps** — the theta logger reprints its whole history
   each line with no delimiters; the analysis takes the longest run per file.
3. **PuTTY log headers** are embedded in several CSVs and must be stripped.
4. **`-1.00` sentinel** appears in the switching theta log — confirm its meaning
   against the firmware before treating those rows as data.
5. **Outlier-dominated means** — variance means are skewed by rare spikes;
   conclusions lean on medians.
6. **Two-sensor steering** — steering uses only sensors 1 and 3.

## Roadmap

**Controller** — full 5-sensor weighted position (or PID); **debounce the switch**
(hysteresis / minimum dwell — the likely source of the wobble); or fuse the two
arrays by inverse-variance weight instead of a hard pick.
**Sensing** — low-pass filter on calibrated readings; investigate the noisy
middle sensor.
**Odometry** — calibrate `wheelRadius` / `wheelSeparation` /
`encoderCountsPerRevolution` against ground truth (the heading formula was
corrected in this refactor).
**Data** — log with timestamps and a fixed rate; emit clean CSV; collect more
and longer runs (especially switching).
**Tooling** — a PlatformIO project / CI `arduino-cli compile` check; host-side
unit tests for the pure logic (a stubbed Arduino API already syntax-verifies the
headers under `g++ -Wall -Wextra`).

## Firmware fixes in this refactor

Behaviour of the core algorithm is preserved; these remove real bugs and
undefined behaviour. Highlights: added the missing `motors.initialise()`; removed
a stray brace and an always-on `while(true)` block that stranded `loop()`; made
`calculateVariance()` actually return; replaced variable-length arrays with fixed
ones; fixed the analog sampling loop that discarded all but the last sample; made
analog/digital average-variance consistent so the switch compares like with like;
de-collided the two headers' pin macros (`A_`/`D_` prefixes); fixed a `thetaLog`
buffer overflow; and corrected the differential-drive heading update
(`Δθ = (dR − dL) / wheelSeparation`, previously divided by `2×` — **re-baseline
before comparing new theta to old runs**).

## Credits & licence

The full write-up is in [`docs/line-following.pdf`](docs/line-following.pdf):
*"Investigating Line-Following Robot Navigation through Variance-Based Sensor
Switching: A Hybrid Approach to Optimizing Sensor Performance"* — authors
**nz20469**, **xi20942**. Encoder/odometry scaffolding and inline guidance credit
Paul O'Dowd (University of Bristol course material).

No licence file is included yet — add one (e.g. MIT) if you intend others to
reuse this code.
