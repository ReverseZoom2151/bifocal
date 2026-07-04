// Bifocal offline replay / simulation harness.
// -----------------------------------------------------------------------------
// Purpose
//   The original study compared the analog, digital, and variance-based
//   switching strategies on separate physical runs. Those runs were not
//   comparable: different headings, unequal sample counts, different noise
//   realisations. This harness removes that weakness by running ALL FOUR
//   strategies on ONE identical synthetic input stream, so the only thing that
//   differs between them is the control law. That makes the tracking-accuracy
//   vs steering-smoothness trade-off directly measurable.
//
// What it does
//   1. Generates a synthetic ground-truth line trajectory over T ticks
//      (sinusoid plus step segments: straights and curves).
//   2. Pre-generates a single deterministic noise/spike realisation from a
//      fixed PRNG seed, so every strategy is disturbed by the same random
//      sequence (apples to apples).
//   3. At each tick, for each strategy, synthesizes two 5-element calibrated
//      sensor arrays (an ANALOG view: continuous bump, more Gaussian noise,
//      occasional large spikes; a DIGITAL view: quantized bump, less noise,
//      rare spikes) centred on the line position the robot currently sees.
//   4. Computes a rolling per-array variance over a short window (same idea as
//      the firmware calculateAverageVariance()).
//   5. Runs four controllers (analog-only, digital-only, variance switching
//      with hysteresis + dwell, inverse-variance fusion). Each reuses
//      linePosition5() and a PID_c from the firmware steering.h and drives a
//      simple 1D robot model.
//   6. Prints a labelled metrics table (steering-derivative RMS, IAE, mean and
//      final absolute error, switches per 100 ticks).
//
// Reuses firmware/line_following/steering.h unchanged (linePosition5, PID_c).
// Standard C++11, no external libraries, fully deterministic.

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <random>
#include <vector>
#include <string>

// steering.h uses the Arduino constrain() macro (in PID_c::update). Pull in the
// host stub that defines it, then the firmware header itself.
#include "arduino_stub.h"
#include "steering.h"

// ---- Simulation constants --------------------------------------------------

static const int   T             = 600;    // number of control ticks
static const float DT            = 0.1f;    // control period in seconds (10 Hz)

static const int   VAR_WINDOW    = 5;       // rolling variance window (ticks)

// Sensor synthesis. Index 0 = left .. 4 = right. A reflectance bump is centred
// on the line position the robot currently sees, expressed as a sensor index.
static const float BUMP_SIGMA    = 1.0f;    // width of the reflectance bump

static const float ANALOG_NOISE  = 0.05f;   // analog Gaussian noise std
static const float ANALOG_SPIKE_P= 0.02f;   // analog spike probability per cell
static const float ANALOG_SPIKE_M= 0.8f;    // analog spike magnitude

static const float DIGITAL_NOISE = 0.02f;   // digital pre-threshold noise std
static const float DIGITAL_SPIKE_P = 0.004f;// digital spike probability per cell
static const float DIGITAL_THRESH= 0.5f;    // digital quantisation threshold

// Robot model. The turn term W steers the robot laterally toward the line.
static const float STEER_GAIN    = 1.2f;    // lateral response per unit turn

// Switching debounce (mirrors the firmware METHOD_SWITCHING settings).
static const float SWITCH_MARGIN   = 0.0005f;
static const int   SWITCH_DWELL_TICKS = 3;   // 300 ms at 100 ms/tick

// Inverse-variance fusion guard (mirrors firmware FUSION_EPS).
static const float FUSION_EPS = 0.0001f;

// ---- Ground-truth line trajectory ------------------------------------------
// A sinusoid (gentle continuous curve) plus additive step segments (abrupt
// lateral offsets, e.g. a sharp corner) so the input contains both straight and
// curved sections. Kept inside [-1, 1].
static float groundTruth(int t) {
  float base = 0.55f * std::sin(2.0f * 3.14159265f * (float)t / 130.0f);
  float step = 0.0f;
  if (t >= 150 && t < 210) step += 0.30f;   // step to the right
  if (t >= 320 && t < 400) step -= 0.35f;   // step to the left
  if (t >= 470 && t < 520) step += 0.25f;   // shorter step to the right
  float v = base + step;
  if (v >  1.0f) v =  1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

// ---- Sensor synthesis ------------------------------------------------------
// Convert a seen line position in [-1, 1] to a sensor-index centre in [0, 4].
static float posToCentre(float pos) {
  if (pos >  1.0f) pos =  1.0f;
  if (pos < -1.0f) pos = -1.0f;
  return (pos + 1.0f) * 2.0f;   // -1 -> 0, 0 -> 2, +1 -> 4
}

// Build the ANALOG calibrated array for a seen position, applying the shared
// pre-generated noise/spike realisation for this tick.
static void synthAnalog(float pos, const float noise[5], const float spike[5],
                        float out[5]) {
  float centre = posToCentre(pos);
  for (int i = 0; i < 5; i++) {
    float d = (float)i - centre;
    float bump = std::exp(-(d * d) / (2.0f * BUMP_SIGMA * BUMP_SIGMA));
    float v = bump + noise[i] + spike[i];
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    out[i] = v;
  }
}

// Build the DIGITAL calibrated array: same bump, less noise, then quantized to
// {0, 1} at a threshold, with rare cell spikes forced high.
static void synthDigital(float pos, const float noise[5], const float spikeHigh[5],
                         float out[5]) {
  float centre = posToCentre(pos);
  for (int i = 0; i < 5; i++) {
    float d = (float)i - centre;
    float bump = std::exp(-(d * d) / (2.0f * BUMP_SIGMA * BUMP_SIGMA));
    float v = bump + noise[i];
    float q = (v >= DIGITAL_THRESH) ? 1.0f : 0.0f;
    if (spikeHigh[i] > 0.5f) q = 1.0f;   // rare stuck-high spike
    out[i] = q;
  }
}

// ---- Rolling variance ------------------------------------------------------
// Mean over the 5 sensors of each sensor's variance across the last VAR_WINDOW
// ticks. Same reduction as the firmware calculateAverageVariance(), but the
// window is temporal (consecutive ticks) rather than repeated reads.
struct VarWindow {
  std::vector< std::vector<float> > hist;   // ring of recent calibrated arrays
  int filled;
  VarWindow() : filled(0) {}

  float push(const float cal[5]) {
    std::vector<float> row(5);
    for (int i = 0; i < 5; i++) row[i] = cal[i];
    hist.push_back(row);
    if ((int)hist.size() > VAR_WINDOW) hist.erase(hist.begin());
    if (filled < VAR_WINDOW) filled++;

    int n = (int)hist.size();
    float total = 0.0f;
    for (int s = 0; s < 5; s++) {
      float mean = 0.0f;
      for (int k = 0; k < n; k++) mean += hist[k][s];
      mean /= (float)n;
      float var = 0.0f;
      for (int k = 0; k < n; k++) {
        float e = hist[k][s] - mean;
        var += e * e;
      }
      var /= (float)n;
      total += var;
    }
    return total / 5.0f;
  }
};

// ---- Per-strategy controller state -----------------------------------------
enum Strategy { S_ANALOG = 0, S_DIGITAL = 1, S_SWITCH = 2, S_FUSION = 3 };

struct Controller {
  Strategy    strat;
  PID_c       pid;
  VarWindow   va;          // rolling variance, analog array
  VarWindow   vd;          // rolling variance, digital array
  float       robotPos;    // robot lateral offset (tracks the true line)
  float       lastW;       // previous turn term, for derivative RMS
  bool        haveLastW;

  // switching debounce state
  int         choice;      // 0 analog, 1 digital
  int         lastSwitchTick;
  int         switches;    // count of choice changes

  // accumulated metrics
  double      sumSqDeriv;  // sum of (W_t - W_{t-1})^2
  int         derivCount;
  double      iae;         // integral of |tracking error| dt
  double      sumAbsErr;   // sum |tracking error|
  double      finalAbsErr;

  Controller(Strategy s)
    : strat(s), robotPos(0.0f), lastW(0.0f), haveLastW(false),
      choice(0), lastSwitchTick(-100000), switches(0),
      sumSqDeriv(0.0), derivCount(0), iae(0.0), sumAbsErr(0.0),
      finalAbsErr(0.0) {}
};

// One control tick for a strategy. Uses the shared per-tick noise realisation.
static void stepController(Controller& c, int t, float trueLine,
                           const float aNoise[5], const float aSpike[5],
                           const float dNoise[5], const float dSpikeHigh[5]) {

  // What the robot currently sees: the line offset relative to its own lateral
  // position. This is the quantity the sensor arrays are built around.
  float rel = trueLine - c.robotPos;

  float aCal[5];
  float dCal[5];
  synthAnalog(rel, aNoise, aSpike, aCal);
  synthDigital(rel, dNoise, dSpikeHigh, dCal);

  float varA = c.va.push(aCal);
  float varD = c.vd.push(dCal);

  float eA = linePosition5(aCal);
  float eD = linePosition5(dCal);

  // Select or fuse the line error per strategy (mirrors line_following.ino).
  float lineErr = 0.0f;
  if (c.strat == S_ANALOG) {
    lineErr = eA;
  } else if (c.strat == S_DIGITAL) {
    lineErr = eD;
  } else if (c.strat == S_FUSION) {
    float wA = 1.0f / (varA + FUSION_EPS);
    float wD = 1.0f / (varD + FUSION_EPS);
    lineErr = (wA * eA + wD * eD) / (wA + wD);
  } else { // S_SWITCH: hysteresis + dwell
    if (t - c.lastSwitchTick >= SWITCH_DWELL_TICKS) {
      if (c.choice == 0) {
        if (varD + SWITCH_MARGIN < varA) { c.choice = 1; c.lastSwitchTick = t; c.switches++; }
      } else {
        if (varA + SWITCH_MARGIN < varD) { c.choice = 0; c.lastSwitchTick = t; c.switches++; }
      }
    }
    lineErr = (c.choice == 0) ? eA : eD;
  }

  // Turn term from the PID, same as STEER_PID in the firmware.
  float W = c.pid.update(lineErr, DT);
  if (W >  1.0f) W =  1.0f;
  if (W < -1.0f) W = -1.0f;

  // Steering-derivative energy (jerkiness of the turn command).
  if (c.haveLastW) {
    double d = (double)W - (double)c.lastW;
    c.sumSqDeriv += d * d;
    c.derivCount++;
  }
  c.lastW = W;
  c.haveLastW = true;

  // 1D robot model: the turn steers the robot laterally toward the line.
  c.robotPos += STEER_GAIN * W * DT;

  // Tracking error is the true line offset from the robot centre.
  float trackErr = trueLine - c.robotPos;
  float absErr = std::fabs(trackErr);
  c.iae += (double)absErr * DT;
  c.sumAbsErr += absErr;
  c.finalAbsErr = absErr;
}

int main(int argc, char** argv) {

  // Deterministic seed. Constant by default, overridable as argv[1] so runs
  // stay reproducible and never depend on wall-clock time.
  unsigned int seed = 12345u;
  if (argc > 1) seed = (unsigned int)std::strtoul(argv[1], 0, 10);

  // The shared Arduino host stub defines a file-static Serial that steering.h
  // does not use here. Reference it once so -Wall stays clean without editing
  // the shared test stub.
  (void)sizeof(Serial);

  std::mt19937 rng(seed);
  std::normal_distribution<float>       gaussA(0.0f, ANALOG_NOISE);
  std::normal_distribution<float>       gaussD(0.0f, DIGITAL_NOISE);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);

  // Pre-generate ONE shared noise/spike realisation for all T ticks so every
  // strategy is driven by the identical random disturbance.
  std::vector< std::vector<float> > aNoise(T, std::vector<float>(5));
  std::vector< std::vector<float> > aSpike(T, std::vector<float>(5));
  std::vector< std::vector<float> > dNoise(T, std::vector<float>(5));
  std::vector< std::vector<float> > dSpikeHigh(T, std::vector<float>(5));

  for (int t = 0; t < T; t++) {
    for (int i = 0; i < 5; i++) {
      aNoise[t][i] = gaussA(rng);
      aSpike[t][i] = (uni(rng) < ANALOG_SPIKE_P) ? ANALOG_SPIKE_M : 0.0f;
      dNoise[t][i] = gaussD(rng);
      dSpikeHigh[t][i] = (uni(rng) < DIGITAL_SPIKE_P) ? 1.0f : 0.0f;
    }
  }

  Controller controllers[4] = {
    Controller(S_ANALOG),
    Controller(S_DIGITAL),
    Controller(S_SWITCH),
    Controller(S_FUSION)
  };

  // Run every strategy over the same trajectory and the same noise realisation.
  for (int t = 0; t < T; t++) {
    float trueLine = groundTruth(t);
    for (int k = 0; k < 4; k++) {
      stepController(controllers[k], t, trueLine,
                     &aNoise[t][0], &aSpike[t][0],
                     &dNoise[t][0], &dSpikeHigh[t][0]);
    }
  }

  // ---- Report --------------------------------------------------------------
  const char* names[4] = { "analog-only", "digital-only", "switching", "fusion" };

  printf("Bifocal replay harness -- identical-input strategy comparison\n");
  printf("seed=%u  ticks=%d  dt=%.2fs  var_window=%d\n\n", seed, T, DT, VAR_WINDOW);
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

  for (int k = 0; k < 4; k++) {
    Controller& c = controllers[k];
    double steerRMS = (c.derivCount > 0)
                      ? std::sqrt(c.sumSqDeriv / (double)c.derivCount) : 0.0;
    double meanAbs  = c.sumAbsErr / (double)T;
    double swPer100 = (c.strat == S_SWITCH)
                      ? (100.0 * (double)c.switches / (double)T) : 0.0;

    if (c.strat == S_SWITCH) {
      printf("%-14s %10.5f %10.4f %11.5f %10.5f %11.2f\n",
             names[k], steerRMS, c.iae, meanAbs, c.finalAbsErr, swPer100);
    } else {
      printf("%-14s %10.5f %10.4f %11.5f %10.5f %11s\n",
             names[k], steerRMS, c.iae, meanAbs, c.finalAbsErr, "-");
    }
  }

  printf("\nRaw switch count (switching strategy): %d over %d ticks\n",
         controllers[S_SWITCH].switches, T);

  return 0;
}
