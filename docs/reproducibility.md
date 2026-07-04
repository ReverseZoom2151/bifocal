# Reproducibility

This document explains how to reproduce the host-buildable, verifiable parts of
Bifocal with a single command, either natively or inside a container. The runner
is [`reproduce.sh`](../reproduce.sh) at the repository root.

## What reproduce.sh does

`reproduce.sh` rebuilds and re-verifies everything that does not require the
physical robot or the Arduino AVR toolchain. It runs these steps, in order, and
reports PASS, SKIP, or FAIL for each:

1. Host unit tests: `cd tests && make test`. Builds the stubbed-Arduino test
   suites under `g++ -Wall` and runs them.
2. Offline replay simulation: `cd sim && make run`. Builds and runs the replay
   harness that compares all four strategies on identical seeded inputs.
3. Python figures and statistics: sets up a Python environment and runs the
   analysis scripts to regenerate the figures in `gallery/`.
4. Firmware compile (optional): only if `arduino-cli` is installed. Skipped
   with a clear message otherwise.
5. WebAssembly sim build (optional): only if `emcc` is installed. Skipped
   otherwise.

The script uses `command -v` guards for the optional tools and file-existence
guards for optional analysis scripts, so a missing optional tool or script is a
SKIP, not a failure. It exits non-zero only if a step that is expected to work
(the tests, the sim, or the required Python figures) failed.

### Optional analysis scripts

If `analysis/train_policy.py` or `analysis/pareto.py` exist, they are run after
the three required analysis scripts and their result is reported as a
non-blocking optional step. If they are absent they are reported as SKIP. The
same applies to an optional `sim/sweep` parameter sweep (`sim/sweep.py`,
`sim/sweep`, or a `make sweep` target) if one is present.

## Requirements

Native run:

- A POSIX shell (`sh`). On Windows use Git Bash or MSYS2.
- `g++` (C++11) and `make` for the tests and the sim.
- Python 3 with `pip`, plus the packages in
  [`requirements.txt`](../requirements.txt): pandas, numpy, matplotlib, seaborn,
  and optionally scipy. The runner installs these into a local `.venv` by
  default (needs network the first time), and falls back to the system
  interpreter if a virtual environment or `pip install` is not possible.

Optional, only for the guarded steps:

- `arduino-cli` plus the `arduino:avr` core, to compile the microcontroller
  firmware (step 4).
- `emcc` (Emscripten), to build a WebAssembly copy of the sim (step 5).

## Running natively

From the repository root:

```sh
sh reproduce.sh
```

To skip the virtual environment and use the system Python (for example when the
dependencies are already installed and there is no network):

```sh
BIFOCAL_NO_VENV=1 sh reproduce.sh
```

## Running with Docker

The [`Dockerfile`](../Dockerfile) builds a small `python:3.12-slim` image with
`g++` and `make`, installs the Python requirements, copies the repository, and
runs `reproduce.sh`.

```sh
docker build -t bifocal-repro .
docker run --rm bifocal-repro
```

To copy the regenerated figures out of the container:

```sh
cid=$(docker create bifocal-repro)
docker cp "$cid":/app/gallery ./gallery-from-container
docker rm "$cid"
```

The image sets `BIFOCAL_NO_VENV=1` because the dependencies are already
installed system-wide inside it, so no venv is built at run time.

## Expected outputs

Unit tests (step 1): both suites pass, for example

```text
test_kinematics summary: 12 checks, 12 passed, 0 failed
test_variance   summary: 21 checks, 21 passed, 0 failed
ALL TEST SUITES PASSED
```

Sim replay (step 2): a deterministic (seeded) metrics table comparing the four
strategies, for example

```text
strategy         steerRMS        IAE  meanAbsErr   finalErr  switch/100
analog-only       0.39454     8.0557     0.13426    0.19321           -
digital-only      0.78931     5.6852     0.09475    0.07440           -
switching         0.39305     6.2730     0.10455    0.08510       14.17
fusion            0.31214     6.1418     0.10236    0.08592           -
```

Figures and statistics (step 3): the following PNGs are regenerated in
`gallery/`, and `analysis/statistics.py` and `analysis/sensor_noise.py` also
print their tables to the terminal.

| File | Produced by |
|------|-------------|
| `gallery/variance_comparison.png` | `analysis/make_gallery.py` |
| `gallery/theta_stability.png` | `analysis/make_gallery.py` |
| `gallery/calibration_contrast.png` | `analysis/make_gallery.py` |
| `gallery/sensor_noise.png` | `analysis/sensor_noise.py` |
| `gallery/statistics.png` | `analysis/statistics.py` |

If the optional `analysis/train_policy.py` or `analysis/pareto.py` are present
they produce their own additional output (for example a learned-policy header or
a Pareto figure); if absent they are skipped.

The run ends with a summary block listing every step and its PASS / SKIP / FAIL
status.

## Note on the AVR toolchain

The microcontroller firmware in `firmware/` targets the ATmega32U4 and needs the
Arduino AVR toolchain (`arduino-cli` plus the `arduino:avr` core, or PlatformIO)
to compile and flash. That toolchain is large and hardware-oriented and is not
included in the Docker image or assumed by the native runner. The firmware
compile is therefore an optional, guarded step: if `arduino-cli` is not
installed it is skipped with a clear message, and the rest of the reproduction
still completes. To also verify the firmware build, install `arduino-cli` and
the `arduino:avr` core (see the README and `docs/build-and-ci.md`) and re-run
`reproduce.sh`, or build it separately on a machine that has the toolchain.
