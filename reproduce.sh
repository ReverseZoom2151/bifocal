#!/bin/sh
# ==========================================================================
# Bifocal one-command reproducibility runner.
#
# Rebuilds and verifies every host-buildable, verifiable part of the project:
#
#   1. Host unit tests            (cd tests && make test)
#   2. Offline replay simulation  (cd sim && make run)
#   3. Python figures and stats   (analysis/*.py -> gallery/*.png)
#   4. Firmware compile           (optional; only if arduino-cli is installed)
#   5. WebAssembly sim build      (optional; only if emcc is installed)
#
# Each step reports PASS, SKIP, or FAIL and the script prints a summary at the
# end. It exits non-zero only if a step that is expected to work FAILED, so an
# absent optional tool (arduino-cli, emcc) is a SKIP, not a failure.
#
# The firmware itself needs the Arduino AVR toolchain, which is not part of a
# plain container. Steps 1 to 3 need only g++, make, and Python. See
# docs/reproducibility.md for details.
#
# Usage:
#   sh reproduce.sh
#   BIFOCAL_NO_VENV=1 sh reproduce.sh    # use system Python, skip the venv
#
# POSIX sh. Runs on Linux, macOS, and Windows (Git Bash / MSYS2).
# ==========================================================================

set -eu

# --- locate the repo root (this script's directory) -----------------------
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

# --- summary bookkeeping --------------------------------------------------
# POSIX sh has no arrays, so accumulate "name<TAB>status" lines in a string.
SUMMARY=""
REQUIRED_FAILED=0

record() {
  # record <step-name> <status>
  SUMMARY="${SUMMARY}${1}	${2}
"
}

banner() {
  printf '\n'
  printf '============================================================\n'
  printf '  %s\n' "$1"
  printf '============================================================\n'
}

info() { printf '  %s\n' "$1"; }

# --- pick a working Python interpreter ------------------------------------
find_python() {
  for c in python3 python py; do
    if command -v "$c" >/dev/null 2>&1; then
      if "$c" -c 'import sys; sys.exit(0)' >/dev/null 2>&1; then
        printf '%s' "$c"
        return 0
      fi
    fi
  done
  return 1
}

# --- venv python path is Scripts/ on Windows, bin/ elsewhere --------------
venv_python() {
  if [ -x ".venv/bin/python" ]; then
    printf '%s' ".venv/bin/python"
  elif [ -x ".venv/Scripts/python.exe" ]; then
    printf '%s' ".venv/Scripts/python.exe"
  elif [ -f ".venv/Scripts/python.exe" ]; then
    printf '%s' ".venv/Scripts/python.exe"
  else
    return 1
  fi
}

# ==========================================================================
# STEP 1: host unit tests
# ==========================================================================
banner "STEP 1/5  Host unit tests  (tests/make test)"
if ! command -v make >/dev/null 2>&1; then
  info "make not found; cannot build the tests."
  record "unit tests" "FAIL (make missing)"
  REQUIRED_FAILED=1
elif ! command -v g++ >/dev/null 2>&1; then
  info "g++ not found; cannot build the tests."
  record "unit tests" "FAIL (g++ missing)"
  REQUIRED_FAILED=1
elif ( cd tests && make test ); then
  info "unit tests passed."
  record "unit tests" "PASS"
else
  info "unit tests FAILED."
  record "unit tests" "FAIL"
  REQUIRED_FAILED=1
fi

# ==========================================================================
# STEP 2: offline replay simulation
# ==========================================================================
banner "STEP 2/5  Offline replay simulation  (sim/make run)"
if ! command -v make >/dev/null 2>&1 || ! command -v g++ >/dev/null 2>&1; then
  info "make or g++ not found; cannot build the sim."
  record "sim replay" "FAIL (toolchain missing)"
  REQUIRED_FAILED=1
elif ( cd sim && make run ); then
  info "sim built and ran."
  record "sim replay" "PASS"
else
  info "sim FAILED."
  record "sim replay" "FAIL"
  REQUIRED_FAILED=1
fi

# Optional parameter sweep, only if a sibling created it (guarded).
if [ -f "sim/sweep.py" ] || [ -f "sim/sweep" ] || [ -f "sim/sweep.cpp" ]; then
  banner "STEP 2b   Optional sim parameter sweep"
  if [ -f "sim/sweep.py" ] && PY=$(find_python); then
    if "$PY" sim/sweep.py; then
      info "sim sweep ran."
      record "sim sweep (optional)" "PASS"
    else
      info "sim sweep present but failed; continuing."
      record "sim sweep (optional)" "FAIL (non-blocking)"
    fi
  elif ( cd sim && make sweep ) 2>/dev/null; then
    info "sim sweep target ran."
    record "sim sweep (optional)" "PASS"
  else
    info "sim sweep present but could not be run; continuing."
    record "sim sweep (optional)" "SKIP"
  fi
else
  record "sim sweep (optional)" "SKIP (not present)"
fi

# ==========================================================================
# STEP 3: Python environment, figures and statistics
# ==========================================================================
banner "STEP 3/5  Python figures and statistics  (analysis/*.py)"

PY=""
if PY=$(find_python); then
  info "using base interpreter: $PY"
else
  info "no working Python interpreter found."
  record "python figures" "FAIL (no python)"
  REQUIRED_FAILED=1
fi

if [ -n "$PY" ]; then
  RUNPY="$PY"

  # Prefer an isolated virtual environment unless disabled.
  if [ "${BIFOCAL_NO_VENV:-0}" = "1" ]; then
    info "BIFOCAL_NO_VENV=1 set; using the system interpreter."
  else
    if [ ! -d ".venv" ]; then
      info "creating virtual environment in .venv ..."
      if ! "$PY" -m venv .venv >/dev/null 2>&1; then
        info "could not create a venv (continuing with system Python)."
      fi
    fi
    if VPY=$(venv_python); then
      info "installing requirements into .venv (needs network the first time) ..."
      if "$VPY" -m pip install --disable-pip-version-check -q -r requirements.txt >/dev/null 2>&1; then
        RUNPY="$VPY"
        info "using venv interpreter: $VPY"
      else
        info "pip install failed (offline?); falling back to system Python."
      fi
    fi
  fi

  # Verify the hard dependencies are importable before running the scripts.
  if "$RUNPY" -c "import numpy, matplotlib" >/dev/null 2>&1; then
    figs_ok=1

    # Required analysis scripts (always present).
    for script in make_gallery.py sensor_noise.py statistics.py; do
      if [ -f "analysis/$script" ]; then
        info "running analysis/$script ..."
        if "$RUNPY" "analysis/$script"; then
          info "analysis/$script done."
        else
          info "analysis/$script FAILED."
          figs_ok=0
        fi
      else
        info "analysis/$script not found; skipping."
      fi
    done

    # Optional analysis scripts (may be added by sibling work). Guarded so the
    # runner still works whether or not they exist.
    for script in train_policy.py pareto.py; do
      if [ -f "analysis/$script" ]; then
        banner "STEP 3b   Optional analysis: analysis/$script"
        if "$RUNPY" "analysis/$script"; then
          info "analysis/$script done."
          record "analysis/$script (optional)" "PASS"
        else
          info "analysis/$script present but failed; continuing."
          record "analysis/$script (optional)" "FAIL (non-blocking)"
        fi
      else
        record "analysis/$script (optional)" "SKIP (not present)"
      fi
    done

    if [ "$figs_ok" = "1" ]; then
      record "python figures" "PASS"
    else
      record "python figures" "FAIL"
      REQUIRED_FAILED=1
    fi
  else
    info "numpy/matplotlib not importable; cannot generate figures."
    info "install them with: $RUNPY -m pip install -r requirements.txt"
    record "python figures" "FAIL (deps missing)"
    REQUIRED_FAILED=1
  fi
fi

# ==========================================================================
# STEP 4: firmware compile (optional, needs the Arduino AVR toolchain)
# ==========================================================================
banner "STEP 4/5  Firmware compile  (optional, arduino-cli)"
if command -v arduino-cli >/dev/null 2>&1; then
  info "arduino-cli found; compiling firmware/line_following ..."
  arduino-cli core install arduino:avr >/dev/null 2>&1 || true
  if arduino-cli compile --fqbn arduino:avr:leonardo firmware/line_following; then
    info "firmware compiled."
    record "firmware compile (optional)" "PASS"
  else
    info "firmware compile FAILED."
    record "firmware compile (optional)" "FAIL (non-blocking)"
  fi
else
  info "arduino-cli not installed; skipping the firmware compile."
  info "install it and the arduino:avr core to build the AVR firmware."
  record "firmware compile (optional)" "SKIP (arduino-cli missing)"
fi

# ==========================================================================
# STEP 5: WebAssembly build of the sim (optional, needs emcc)
# ==========================================================================
banner "STEP 5/5  WebAssembly sim build  (optional, emcc)"
if command -v emcc >/dev/null 2>&1; then
  info "emcc found; building a WebAssembly copy of the replay sim ..."
  if emcc -std=c++11 -O2 -I firmware/line_following -I tests \
       sim/replay.cpp -o sim/replay.js; then
    info "wasm sim built (sim/replay.js, sim/replay.wasm)."
    record "wasm sim build (optional)" "PASS"
  else
    info "wasm sim build FAILED."
    record "wasm sim build (optional)" "FAIL (non-blocking)"
  fi
else
  info "emcc not installed; skipping the WebAssembly sim build."
  record "wasm sim build (optional)" "SKIP (emcc missing)"
fi

# ==========================================================================
# SUMMARY
# ==========================================================================
banner "SUMMARY"
printf '%b' "$SUMMARY" | while IFS='	' read -r name status; do
  [ -n "$name" ] || continue
  printf '  %-32s %s\n' "$name" "$status"
done

printf '\n'
if [ "$REQUIRED_FAILED" -eq 0 ]; then
  info "All required steps passed. Optional steps may have been skipped above."
  info "Regenerated figures are in gallery/."
  exit 0
else
  info "One or more required steps FAILED (see above)."
  exit 1
fi
