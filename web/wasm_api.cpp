// WebAssembly entry point for the Bifocal simulation.
// -----------------------------------------------------------------------------
// A thin C-linkage wrapper over sim/harness.h so a browser can run one trial and
// read back the four strategies' metrics. Built with emscripten (see build.sh).
//
// It also compiles with a normal host g++ (the emscripten-specific bits are
// guarded by __EMSCRIPTEN__), which is only used as a sanity check that the API
// wiring builds. The actual .wasm/.js output must come from emcc.
//
// Reuses sim/harness.h (bifocal::runTrial). Standard C++11.

#include "harness.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define BIFOCAL_KEEPALIVE EMSCRIPTEN_KEEPALIVE
#else
#define BIFOCAL_KEEPALIVE
#endif

using namespace bifocal;

// Output layout: NUM_STRATEGIES strategies, 5 doubles each, in strategy order
// (analog-only, digital-only, switching, fusion):
//   [0] steerRMS  [1] IAE  [2] meanAbsErr  [3] finalErr  [4] switchPer100
// The caller (JS) reads these back from the returned heap pointer.
static double g_out[NUM_STRATEGIES * 5];

extern "C" {

// Run one deterministic trial and return a pointer to the metrics buffer above.
BIFOCAL_KEEPALIVE
double* bifocal_run(double switchMargin, int switchDwell, double Kp,
                    double analogNoise, double digitalNoise,
                    unsigned int seed) {
  // Reference the shared static Serial once so -Wall stays clean.
  (void)sizeof(Serial);

  Params p;
  p.switchMargin = (float)switchMargin;
  p.switchDwell  = switchDwell;
  p.Kp           = (float)Kp;
  p.analogNoise  = (float)analogNoise;
  p.digitalNoise = (float)digitalNoise;

  TrialResult r = runTrial(p, seed);
  for (int k = 0; k < NUM_STRATEGIES; k++) {
    const Metrics& m = r.m[k];
    g_out[k * 5 + 0] = m.steerRMS;
    g_out[k * 5 + 1] = m.iae;
    g_out[k * 5 + 2] = m.meanAbsErr;
    g_out[k * 5 + 3] = m.finalErr;
    g_out[k * 5 + 4] = m.switchPer100;
  }
  return g_out;
}

// Number of strategies (so the JS side does not hard-code it).
BIFOCAL_KEEPALIVE
int bifocal_num_strategies() { return NUM_STRATEGIES; }

// Number of ticks per trial, for display.
BIFOCAL_KEEPALIVE
int bifocal_ticks() { return HARNESS_T; }

} // extern "C"
