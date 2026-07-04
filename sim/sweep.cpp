// Bifocal parameter sweep with Monte Carlo aggregation.
// -----------------------------------------------------------------------------
// Sweeps the switching strategy's SWITCH margin and dwell (and, as a third
// dimension, the PID Kp gain) across a grid. Every grid point is evaluated over
// a fixed list of seeds (Monte Carlo), and the mean and standard deviation of
// steerRMS and IAE across those seeds are written to a CSV.
//
// The three parameter-independent strategies (analog-only, digital-only,
// fusion) do not depend on the switch margin or dwell, so they are recorded
// once per Kp as baseline reference rows (with the default margin/dwell).
//
// Everything is deterministic: the seed list is fixed in code, so re-running the
// sweep reproduces the CSV exactly. No wall-clock randomness.
//
// Output: written to sim/sweep_results.csv by default, or to stdout if the
// first argument is "-". Uses sim/harness.h (bifocal::runTrial). Standard C++11.

#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

#include "harness.h"

using namespace bifocal;

// ---- Monte Carlo accumulator for one grid point / strategy -----------------
struct Accum {
  double sRMS, sRMS2;     // sum and sum of squares of steerRMS
  double sIAE, sIAE2;     // sum and sum of squares of IAE
  double sSw,  sSw2;      // sum and sum of squares of switch/100
  int    n;
  Accum() : sRMS(0), sRMS2(0), sIAE(0), sIAE2(0), sSw(0), sSw2(0), n(0) {}

  void add(const Metrics& m) {
    sRMS += m.steerRMS; sRMS2 += m.steerRMS * m.steerRMS;
    sIAE += m.iae;      sIAE2 += m.iae * m.iae;
    sSw  += m.switchPer100; sSw2 += m.switchPer100 * m.switchPer100;
    n++;
  }

  static double mean(double s, int n) { return (n > 0) ? s / (double)n : 0.0; }

  // Sample standard deviation (n-1 denominator). Returns 0 for n < 2.
  static double stddev(double s, double s2, int n) {
    if (n < 2) return 0.0;
    double mu = s / (double)n;
    double var = (s2 - (double)n * mu * mu) / (double)(n - 1);
    if (var < 0.0) var = 0.0;   // guard tiny negative from rounding
    return std::sqrt(var);
  }
};

int main(int argc, char** argv) {

  // Reference the shared static Serial once so -Wall stays clean.
  (void)sizeof(Serial);

  // ---- Fixed Monte Carlo seed list ----------------------------------------
  // A fixed, deterministic list. Re-running reproduces the CSV byte for byte.
  const int   NUM_SEEDS = 40;
  const unsigned int SEED_BASE = 1000u;
  std::vector<unsigned int> seeds;
  for (int i = 0; i < NUM_SEEDS; i++) seeds.push_back(SEED_BASE + (unsigned int)i);

  // ---- Sweep grids ---------------------------------------------------------
  // SWITCH hysteresis margin: from 0 (switch on any variance advantage) up to a
  // large margin (switch only on a big advantage, i.e. rarely).
  const float margins[] = { 0.0f, 0.0002f, 0.0005f, 0.001f, 0.002f, 0.004f, 0.008f };
  const int   nMargin   = (int)(sizeof(margins) / sizeof(margins[0]));

  // SWITCH dwell: minimum ticks between switches (debounce), 1 tick = 100 ms.
  const int   dwells[]  = { 1, 2, 3, 5, 8, 12 };
  const int   nDwell    = (int)(sizeof(dwells) / sizeof(dwells[0]));

  // PID Kp gain (third dimension). The default (1.0) is included.
  const float kps[]     = { 0.8f, 1.0f, 1.2f };
  const int   nKp       = (int)(sizeof(kps) / sizeof(kps[0]));

  Params defaults;   // for the default margin / dwell used by baseline rows

  // ---- Output stream -------------------------------------------------------
  bool toStdout = (argc > 1 && std::strcmp(argv[1], "-") == 0);
  const char* outPath = "sweep_results.csv";
  FILE* out = stdout;
  if (!toStdout) {
    out = std::fopen(outPath, "w");
    if (!out) {
      std::fprintf(stderr, "sweep: cannot open %s for writing\n", outPath);
      return 1;
    }
  }

  // CSV header. steerRMS is the steering-smoothness axis (lower is smoother),
  // IAE is the tracking-accuracy axis (lower tracks better).
  std::fprintf(out,
    "strategy,switch_margin,switch_dwell,Kp,seeds,"
    "steerRMS_mean,steerRMS_std,IAE_mean,IAE_std,"
    "switch_per100_mean,switch_per100_std\n");

  int totalCells = nKp * (3 + nMargin * nDwell);
  int doneCells = 0;

  for (int ik = 0; ik < nKp; ik++) {
    float Kp = kps[ik];

    // Baseline strategies (analog / digital / fusion): margin and dwell do not
    // affect them, so evaluate once at the default margin/dwell for this Kp.
    Accum base[NUM_STRATEGIES];
    Params bp = defaults;
    bp.Kp = Kp;
    for (size_t s = 0; s < seeds.size(); s++) {
      TrialResult r = runTrial(bp, seeds[s]);
      for (int k = 0; k < NUM_STRATEGIES; k++) base[k].add(r.m[k]);
    }
    // Emit the three parameter-independent strategies at the default margin/dwell.
    const int baseStrats[3] = { S_ANALOG, S_DIGITAL, S_FUSION };
    for (int bi = 0; bi < 3; bi++) {
      int k = baseStrats[bi];
      Accum& a = base[k];
      std::fprintf(out,
        "%s,%.6f,%d,%.3f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        strategyName(k), defaults.switchMargin, defaults.switchDwell, Kp, a.n,
        Accum::mean(a.sRMS, a.n), Accum::stddev(a.sRMS, a.sRMS2, a.n),
        Accum::mean(a.sIAE, a.n), Accum::stddev(a.sIAE, a.sIAE2, a.n),
        Accum::mean(a.sSw,  a.n), Accum::stddev(a.sSw,  a.sSw2,  a.n));
    }
    doneCells += 3;

    // Switching strategy over the full margin x dwell grid.
    for (int im = 0; im < nMargin; im++) {
      for (int id = 0; id < nDwell; id++) {
        Params p = defaults;
        p.Kp = Kp;
        p.switchMargin = margins[im];
        p.switchDwell  = dwells[id];

        Accum a;
        for (size_t s = 0; s < seeds.size(); s++) {
          TrialResult r = runTrial(p, seeds[s]);
          a.add(r.m[S_SWITCH]);
        }
        std::fprintf(out,
          "%s,%.6f,%d,%.3f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
          strategyName(S_SWITCH), margins[im], dwells[id], Kp, a.n,
          Accum::mean(a.sRMS, a.n), Accum::stddev(a.sRMS, a.sRMS2, a.n),
          Accum::mean(a.sIAE, a.n), Accum::stddev(a.sIAE, a.sIAE2, a.n),
          Accum::mean(a.sSw,  a.n), Accum::stddev(a.sSw,  a.sSw2,  a.n));

        doneCells++;
        std::fprintf(stderr, "\rsweep: %d/%d cells", doneCells, totalCells);
      }
    }
  }
  std::fprintf(stderr, "\rsweep: %d/%d cells done\n", doneCells, totalCells);

  if (!toStdout) {
    std::fclose(out);
    std::fprintf(stderr, "sweep: wrote %s (%d seeds per cell)\n", outPath, NUM_SEEDS);
  }
  return 0;
}
