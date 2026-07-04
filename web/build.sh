#!/bin/sh
# Build the Bifocal simulation to WebAssembly with emscripten.
#
# Requires emscripten (emcc) on PATH. If you do not have it, install the SDK:
#   git clone https://github.com/emscripten-core/emsdk
#   cd emsdk && ./emsdk install latest && ./emsdk activate latest
#   source ./emsdk_env.sh
#
# Then run this script from the web/ directory:
#   cd web && ./build.sh
#
# It produces web/bifocal.js and web/bifocal.wasm, which web/index.html loads.
# Nothing here depends on wall-clock time; the sim stays deterministic.

set -e

if ! command -v emcc >/dev/null 2>&1; then
  echo "error: emcc not found on PATH. Install emscripten first (see comments above)." >&2
  exit 1
fi

emcc -std=c++11 -O2 \
  -I../sim -I../firmware/line_following -I../tests \
  wasm_api.cpp -o bifocal.js \
  -s MODULARIZE=1 \
  -s EXPORT_NAME=createBifocal \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s "EXPORTED_FUNCTIONS=['_bifocal_run','_bifocal_num_strategies','_bifocal_ticks']" \
  -s "EXPORTED_RUNTIME_METHODS=['cwrap','getValue']"

echo "built: bifocal.js, bifocal.wasm"
