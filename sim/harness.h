// Bifocal offline replay / simulation core (reusable header).
// -----------------------------------------------------------------------------
// This header holds the reusable simulation pieces that used to live inside
// replay.cpp: the synthetic track, the sensor synthesis, the rolling variance,
// the four controllers, and the metric accumulation. They are exposed behind a
// single clean entry point:
//
//   bifocal::TrialResult runTrial(const bifocal::Params& p, unsigned int seed);
//
// runTrial() drives all four strategies (analog-only, digital-only, variance
// switching, inverse-variance fusion) over ONE identical synthetic input stream
// built from the given seed, and returns the metrics for each strategy. It is
// fully deterministic: same params and same seed give the same result, with no
// dependence on wall-clock time.
//
// Params carries the knobs a sweep wants to vary (switch margin, switch dwell,
// PID gains, analog/digital noise levels). Everything else is a fixed constant
// below so the track and the disturbance model stay comparable across a sweep.
//
// Reuses firmware/line_following/steering.h unchanged (linePosition5, PID_c).
// Standard C++11, no external libraries.

#ifndef BIFOCAL_HARNESS_H
#define BIFOCAL_HARNESS_H

#include <cmath>
#include <random>
#include <vector>

// steering.h uses the Arduino constrain() macro (in PID_c::update). Pull in the
// host stub that defines it, then the firmware header itself.
#include "arduino_stub.h"
#include "steering.h"

namespace bifocal {

// ---- Fixed simulation constants --------------------------------------------
// These define the track and the disturbance model. They are intentionally NOT
// part of Params: holding them fixed is what keeps a parameter sweep an
// apples-to-apples comparison of the control laws.

static const int   HARNESS_T          = 600;      // number of control ticks
static const float HARNESS_DT         = 0.1f;     // control period (s), 10 Hz
static const int   HARNESS_VAR_WINDOW = 5;        // rolling variance window (ticks)

static const float BUMP_SIGMA         = 1.0f;     // width of the reflectance bump

static const float ANALOG_SPIKE_P     = 0.02f;    // analog spike probability per cell
static const float ANALOG_SPIKE_M     = 0.8f;     // analog spike magnitude

static const float DIGITAL_SPIKE_P    = 0.004f;   // digital spike probability per cell
static const float DIGITAL_THRESH     = 0.5f;     // digital quantisation threshold

static const float STEER_GAIN         = 1.2f;     // lateral response per unit turn

static const float FUSION_EPS         = 0.0001f;  // inverse-variance fusion guard

// ---- Strategies ------------------------------------------------------------
enum Strategy { S_ANALOG = 0, S_DIGITAL = 1, S_SWITCH = 2, S_FUSION = 3 };
static const int NUM_STRATEGIES = 4;

// ---- Swept parameters ------------------------------------------------------
// Defaults mirror the firmware METHOD_SWITCHING settings and the steering.h
// PID defaults, so runTrial(Params(), 12345) reproduces the original harness.
struct Params {
  float switchMargin;   // hysteresis margin for the switching strategy
  int   switchDwell;    // minimum ticks between switches (debounce)
  float Kp;             // PID proportional gain
  float Ki;             // PID integral gain
  float Kd;             // PID derivative gain
  float analogNoise;    // analog Gaussian noise std
  float digitalNoise;   // digital pre-threshold noise std

  Params()
    : switchMargin(0.0005f), switchDwell(3),
      Kp(1.0f), Ki(0.0f), Kd(0.2f),
      analogNoise(0.05f), digitalNoise(0.02f) {}
};

// ---- Per-strategy metrics --------------------------------------------------
struct Metrics {
  double steerRMS;      // RMS of the steering derivative (turn-command jerkiness)
  double iae;           // integral of absolute tracking error over the run
  double meanAbsErr;    // mean absolute tracking error per tick
  double finalErr;      // absolute tracking error on the last tick
  double switchPer100;  // array switches per 100 ticks (switching strategy only)
  int    switches;      // raw switch count (switching strategy only)

  Metrics()
    : steerRMS(0.0), iae(0.0), meanAbsErr(0.0), finalErr(0.0),
      switchPer100(0.0), switches(0) {}
};

// Result of one trial: metrics for each of the four strategies, indexed by the
// Strategy enum.
struct TrialResult {
  Metrics m[NUM_STRATEGIES];
};

// ---- Ground-truth line trajectory ------------------------------------------
// A sinusoid (gentle continuous curve) plus additive step segments (abrupt
// lateral offsets, e.g. a sharp corner) so the input contains both straight and
// curved sections. Kept inside [-1, 1].
inline float groundTruth(int t) {
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
inline float posToCentre(float pos) {
  if (pos >  1.0f) pos =  1.0f;
  if (pos < -1.0f) pos = -1.0f;
  return (pos + 1.0f) * 2.0f;   // -1 -> 0, 0 -> 2, +1 -> 4
}

// Build the ANALOG calibrated array for a seen position, applying the shared
// pre-generated noise/spike realisation for this tick.
inline void synthAnalog(float pos, const float noise[5], const float spike[5],
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
inline void synthDigital(float pos, const float noise[5], const float spikeHigh[5],
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
// Mean over the 5 sensors of each sensor's variance across the last
// HARNESS_VAR_WINDOW ticks. Same reduction as the firmware
// calculateAverageVariance(), but the window is temporal (consecutive ticks)
// rather than repeated reads.
struct VarWindow {
  std::vector< std::vector<float> > hist;   // ring of recent calibrated arrays
  int filled;
  VarWindow() : filled(0) {}

  float push(const float cal[5]) {
    std::vector<float> row(5);
    for (int i = 0; i < 5; i++) row[i] = cal[i];
    hist.push_back(row);
    if ((int)hist.size() > HARNESS_VAR_WINDOW) hist.erase(hist.begin());
    if (filled < HARNESS_VAR_WINDOW) filled++;

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

  Controller(Strategy s, const Params& p)
    : strat(s), robotPos(0.0f), lastW(0.0f), haveLastW(false),
      choice(0), lastSwitchTick(-100000), switches(0),
      sumSqDeriv(0.0), derivCount(0), iae(0.0), sumAbsErr(0.0),
      finalAbsErr(0.0) {
    pid.Kp = p.Kp;
    pid.Ki = p.Ki;
    pid.Kd = p.Kd;
  }
};

// One control tick for a strategy. Uses the shared per-tick noise realisation
// and the swept params (switch margin / dwell).
inline void stepController(Controller& c, const Params& p, int t, float trueLine,
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
    if (t - c.lastSwitchTick >= p.switchDwell) {
      if (c.choice == 0) {
        if (varD + p.switchMargin < varA) { c.choice = 1; c.lastSwitchTick = t; c.switches++; }
      } else {
        if (varA + p.switchMargin < varD) { c.choice = 0; c.lastSwitchTick = t; c.switches++; }
      }
    }
    lineErr = (c.choice == 0) ? eA : eD;
  }

  // Turn term from the PID, same as STEER_PID in the firmware.
  float W = c.pid.update(lineErr, HARNESS_DT);
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
  c.robotPos += STEER_GAIN * W * HARNESS_DT;

  // Tracking error is the true line offset from the robot centre.
  float trackErr = trueLine - c.robotPos;
  float absErr = std::fabs(trackErr);
  c.iae += (double)absErr * HARNESS_DT;
  c.sumAbsErr += absErr;
  c.finalAbsErr = absErr;
}

// ---- One full trial --------------------------------------------------------
// Runs all four strategies over one identical synthetic input stream built from
// the given seed and params, and returns their metrics. Deterministic.
inline TrialResult runTrial(const Params& p, unsigned int seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float>       gaussA(0.0f, p.analogNoise);
  std::normal_distribution<float>       gaussD(0.0f, p.digitalNoise);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);

  const int T = HARNESS_T;

  // Pre-generate ONE shared noise/spike realisation for all T ticks so every
  // strategy is driven by the identical random disturbance. The draw order here
  // is fixed so results stay reproducible for a given seed.
  std::vector< std::vector<float> > aNoise(T, std::vector<float>(5));
  std::vector< std::vector<float> > aSpike(T, std::vector<float>(5));
  std::vector< std::vector<float> > dNoise(T, std::vector<float>(5));
  std::vector< std::vector<float> > dSpikeHigh(T, std::vector<float>(5));

  for (int t = 0; t < T; t++) {
    for (int i = 0; i < 5; i++) {
      aNoise[t][i]     = gaussA(rng);
      aSpike[t][i]     = (uni(rng) < ANALOG_SPIKE_P)  ? ANALOG_SPIKE_M : 0.0f;
      dNoise[t][i]     = gaussD(rng);
      dSpikeHigh[t][i] = (uni(rng) < DIGITAL_SPIKE_P) ? 1.0f : 0.0f;
    }
  }

  Controller controllers[NUM_STRATEGIES] = {
    Controller(S_ANALOG,  p),
    Controller(S_DIGITAL, p),
    Controller(S_SWITCH,  p),
    Controller(S_FUSION,  p)
  };

  for (int t = 0; t < T; t++) {
    float trueLine = groundTruth(t);
    for (int k = 0; k < NUM_STRATEGIES; k++) {
      stepController(controllers[k], p, t, trueLine,
                     &aNoise[t][0], &aSpike[t][0],
                     &dNoise[t][0], &dSpikeHigh[t][0]);
    }
  }

  TrialResult r;
  for (int k = 0; k < NUM_STRATEGIES; k++) {
    Controller& c = controllers[k];
    Metrics& mk = r.m[k];
    mk.steerRMS   = (c.derivCount > 0)
                    ? std::sqrt(c.sumSqDeriv / (double)c.derivCount) : 0.0;
    mk.iae        = c.iae;
    mk.meanAbsErr = c.sumAbsErr / (double)T;
    mk.finalErr   = c.finalAbsErr;
    mk.switches   = c.switches;
    mk.switchPer100 = (c.strat == S_SWITCH)
                      ? (100.0 * (double)c.switches / (double)T) : 0.0;
  }
  return r;
}

// Human-readable strategy names, indexed by the Strategy enum.
inline const char* strategyName(int k) {
  static const char* names[NUM_STRATEGIES] =
    { "analog-only", "digital-only", "switching", "fusion" };
  if (k < 0 || k >= NUM_STRATEGIES) return "?";
  return names[k];
}

} // namespace bifocal

#endif // BIFOCAL_HARNESS_H
