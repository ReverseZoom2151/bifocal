// Bifocal offline replay / simulation harness (thin main).
// -----------------------------------------------------------------------------
// This is now a thin front end over sim/harness.h. The reusable simulation core
// (synthetic track, sensor synthesis, the four controllers, metric
// accumulation) lives in harness.h behind bifocal::runTrial(). This file only
// parses the seed, runs one trial with the default parameters, and prints the
// labelled metrics table. The output is unchanged from the original harness.
//
// The point of the harness: run ALL FOUR strategies (analog-only, digital-only,
// variance switching with hysteresis + dwell, inverse-variance fusion) on ONE
// identical synthetic input stream, so the only thing that differs between them
// is the control law and the tracking-accuracy vs steering-smoothness trade-off
// becomes directly measurable.
//
// Reuses firmware/line_following/steering.h unchanged (linePosition5, PID_c).
// Standard C++11, no external libraries, fully deterministic (seeded, no
// time-based randomness).

#include <cstdio>
#include <cstdlib>

#include "harness.h"

using namespace bifocal;

int main(int argc, char** argv) {

  // Deterministic seed. Constant by default, overridable as argv[1] so runs
  // stay reproducible and never depend on wall-clock time.
  unsigned int seed = 12345u;
  if (argc > 1) seed = (unsigned int)std::strtoul(argv[1], 0, 10);

  // The shared Arduino host stub defines a file-static Serial that steering.h
  // does not use here. Reference it once so -Wall stays clean without editing
  // the shared test stub.
  (void)sizeof(Serial);

  Params params;                       // firmware default parameters
  TrialResult res = runTrial(params, seed);

  // ---- Report --------------------------------------------------------------
  printf("Bifocal replay harness -- identical-input strategy comparison\n");
  printf("seed=%u  ticks=%d  dt=%.2fs  var_window=%d\n\n",
         seed, HARNESS_T, HARNESS_DT, HARNESS_VAR_WINDOW);
  printf("Metrics:\n");
  printf("  steerRMS  = RMS of the steering-derivative (turn-command jerkiness); lower is smoother\n");
  printf("  IAE       = integral of absolute tracking error over the run; lower tracks better\n");
  printf("  meanAbsErr= mean absolute tracking error per tick\n");
  printf("  finalErr  = absolute tracking error on the last tick\n");
  printf("  switch/100= array switches per 100 ticks (switching strategy only)\n\n");

  printf("%-14s %10s %10s %11s %10s %11s\n",
         "strategy", "steerRMS", "IAE", "meanAbsErr", "finalErr", "switch/100");
  printf("%-14s %10s %10s %11s %10s %11s\n",
         "--------", "--------", "---", "----------", "--------", "----------");

  for (int k = 0; k < NUM_STRATEGIES; k++) {
    const Metrics& m = res.m[k];
    if (k == S_SWITCH) {
      printf("%-14s %10.5f %10.4f %11.5f %10.5f %11.2f\n",
             strategyName(k), m.steerRMS, m.iae, m.meanAbsErr, m.finalErr,
             m.switchPer100);
    } else {
      printf("%-14s %10.5f %10.4f %11.5f %10.5f %11s\n",
             strategyName(k), m.steerRMS, m.iae, m.meanAbsErr, m.finalErr, "-");
    }
  }

  printf("\nRaw switch count (switching strategy): %d over %d ticks\n",
         res.m[S_SWITCH].switches, HARNESS_T);

  return 0;
}
