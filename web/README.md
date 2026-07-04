# Bifocal WebAssembly demo

A small browser demo that compiles the Bifocal simulation core (`sim/harness.h`)
to WebAssembly and runs one deterministic trial in the page, showing the metrics
for all four strategies (analog-only, digital-only, switching, fusion). Sliders
let you change the analog noise, the switch margin, the switch dwell, and the
seed, and the table re-runs the trial on every change.

## Status: the .wasm is not prebuilt

Building the `.wasm` requires the emscripten compiler (`emcc`), which was NOT
available in the environment where this demo was written. So this directory
ships the source and the glue, but not a compiled `bifocal.js` / `bifocal.wasm`.
Nothing here is a fabricated or stubbed binary. You build it yourself with the
one command in `build.sh`.

Until you build it, opening `index.html` shows a clear message telling you to
run `build.sh` first.

## Files

- `wasm_api.cpp`  C-linkage wrapper over `bifocal::runTrial()`. Exposes
  `bifocal_run(...)` which runs one trial and returns a pointer to a metrics
  buffer, plus `bifocal_num_strategies()` and `bifocal_ticks()`. It also
  compiles under a normal host `g++` (the emscripten-only bits are guarded by
  `__EMSCRIPTEN__`) as a wiring sanity check.
- `build.sh`      the exact `emcc` command that produces `bifocal.js` and
  `bifocal.wasm`.
- `index.html`    the page, the sliders, and the JS glue (loads `bifocal.js`,
  calls the exported functions with `cwrap`, reads the metrics back with
  `getValue`).

## How to build

1. Install emscripten (once):

   ```
   git clone https://github.com/emscripten-core/emsdk
   cd emsdk
   ./emsdk install latest
   ./emsdk activate latest
   source ./emsdk_env.sh
   ```

2. Build the module:

   ```
   cd web
   ./build.sh
   ```

   This runs (see `build.sh` for the authoritative version):

   ```
   emcc -std=c++11 -O2 \
     -I../sim -I../firmware/line_following -I../tests \
     wasm_api.cpp -o bifocal.js \
     -s MODULARIZE=1 \
     -s EXPORT_NAME=createBifocal \
     -s ALLOW_MEMORY_GROWTH=1 \
     -s "EXPORTED_FUNCTIONS=['_bifocal_run','_bifocal_num_strategies','_bifocal_ticks']" \
     -s "EXPORTED_RUNTIME_METHODS=['cwrap','getValue']"
   ```

3. Serve the directory over HTTP (browsers will not fetch `.wasm` from a
   `file://` URL):

   ```
   python -m http.server 8000
   ```

   Then open `http://localhost:8000/web/index.html` (or run the server from
   inside `web/` and open `http://localhost:8000/index.html`).

## Determinism

The simulation is fully seeded and has no wall-clock dependency, so the same
slider values and seed always produce the same metrics. The digital noise is
derived as 40 percent of the analog noise to preserve the harness's default
analog/digital ratio (0.05 / 0.02).
