# Build and CI

This document explains the ways to build the Bifocal firmware and how continuous
integration checks it. The firmware is a standard Arduino sketch at
`firmware/line_following/line_following.ino` with its headers in the same folder.
The target microcontroller is an ATmega32U4 (the Pololu 3Pi+ 32U4 control board).

## Target board and the leonardo stand-in

The real board is a Pololu 3Pi+ 32U4, driven by the Pololu A-Star 32U4 core. Its
microcontroller, the ATmega32U4, is the same chip used by the Arduino Leonardo.
Because the Leonardo board definition ships with the stock `arduino:avr` core and
the PlatformIO `atmelavr` platform, this project uses `arduino:avr:leonardo`
(FQBN) and the `leonardo` PlatformIO board as a compile-compatible stand-in. This
keeps the build working without installing the Pololu board index. For flashing
to real hardware, prefer the Pololu A-Star 32U4 board support so that board
specific behavior (bootloader, USB) matches.

## Build path 1: Arduino IDE

1. Install board support for the Pololu A-Star 32U4 / Arduino Leonardo
   (ATmega32U4) core.
2. Open `firmware/line_following/line_following.ino`.
3. Select the board and port, then click Upload.

The sketch folder name must match the `.ino` file name, so keep the folder as
`firmware/line_following/`.

## Build path 2: arduino-cli

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following
arduino-cli upload  --fqbn arduino:avr:leonardo -p COM3 firmware/line_following
```

`arduino-cli compile` treats the folder as the sketch and compiles the `.ino`
together with the neighboring headers. This is exactly what the CI compile job
runs.

## Build path 3: PlatformIO

The project ships a `platformio.ini` at the repository root. PlatformIO normally
expects `.cpp` sources under a `src/` directory, but it also builds Arduino
`.ino` sketches in place: at build time it auto-converts the `.ino` to a `.cpp`,
adding the `Arduino.h` include and function prototypes. To avoid moving any
files, `src_dir` points directly at `firmware/line_following`, so the sketch and
its headers build as one unit.

```bash
pio run                 # build the default (a-star32U4) environment
pio run -e leonardo     # build the leonardo environment
pio run --target upload # build and flash
```

Two environments are defined:

- `a-star32U4`: uses the Pololu A-Star 32U4 board definition, the closest match
  to the 3Pi+ 32U4 hardware.
- `leonardo`: uses the Arduino Leonardo board definition, which is bundled with
  the `atmelavr` platform and needs no extra board index. CI builds this one.

Caveats:

- The `a-star32U4` board id requires the Pololu platform package to be available
  to PlatformIO. If it is not installed, that environment will fail to resolve
  the board; use `pio run -e leonardo` instead, or change the board line in
  `platformio.ini` to `leonardo`. The two are ATmega32U4-compatible.
- No source files are moved for any build path. All three paths compile the
  sketch where it already lives.

## Continuous integration

The workflow at `.github/workflows/ci.yml` runs on every push to `main` or
`master` and on every pull request. It is compile-only; nothing is flashed.

- `arduino-cli` job: installs `arduino-cli` (via `arduino/setup-arduino-cli`),
  installs the `arduino:avr` core, and runs
  `arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following`.
- `platformio` job: installs PlatformIO and runs `pio run -e leonardo`.

Both jobs use the `leonardo` target as the ATmega32U4-compatible stand-in
described above. If you add Pololu-specific board support to CI, switch the FQBN
and PlatformIO environment to the A-Star 32U4 target.
