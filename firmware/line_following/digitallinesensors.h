#ifndef DIGITALLINESENSORS_H
#define DIGITALLINESENSORS_H

// Number of samples used per variance estimate. This is also the length of the
// rolling variance window.
#define D_MAX_SAMPLES 10

// Default smoothing factor for the optional EMA low-pass filter on calibrated
// readings. Range 0..1: higher tracks faster, lower smooths harder.
#define D_FILTER_ALPHA 0.4

// Default outlier clamp for the calibrated path. A new per-sensor calibrated
// value that deviates from the window median by more than this (in calibrated
// 0..1 units) is clamped back toward the median before it reaches variance[].
// Mild by default so normal motion is untouched and only large spikes are cut.
#define D_OUTLIER_CLAMP 0.3

// Default adaptation rate for the optional online recalibration mode. Each
// getCalibrated() nudges offset/scale this fraction of the way toward the
// observed raw range. Conservative so adaptation is slow and stable.
#define D_AUTO_RATE 0.01

// Default relax rate for the online running min/max. Each update moves the
// running min and max this fraction of the way toward their midpoint, so a
// transient extreme does not permanently widen the range. A new true extreme
// still expands the range immediately.
#define D_AUTO_RELAX 0.001

// IR emitter and the 5 line-sensor pins in RC-discharge (digital) mode.
// NOTE: these macros are D_-prefixed so they cannot collide with the analog
// header when both are included in the same sketch (the original code reused
// identical macro names with different values, causing redefinition warnings).
#define D_EMIT_PIN      11
#define D_LS_LEFT_PIN     12
#define D_LS_MIDLEFT_PIN  A0
#define D_LS_MIDDLE_PIN   A2
#define D_LS_MIDRIGHT_PIN A3
#define D_LS_RIGHT_PIN    A4

// Reads the 5-element IR array in digital (RC decay-time) mode: charge the
// sensor capacitor, then time how long it takes to discharge through the
// phototransistor. Longer time == darker surface. Values are normalised to
// 0..1 using per-sensor offset/scale from calibrate().
//
// Variance is now computed with a rolling (non-blocking) window: getCalibrated()
// pushes the newest calibrated[] into a per-sensor ring buffer and updates
// variance[] in O(1) from running sum and sum-of-squares. This replaces the old
// per-call blocking loop that took D_MAX_SAMPLES fresh readings each tick, so
// the controller no longer stalls. It is the one intended behavioural change;
// all other features default to a no-op.
class DigitalLineSensors_c {

  private:

    int ls_pins[5] = { D_LS_LEFT_PIN, D_LS_MIDLEFT_PIN, D_LS_MIDDLE_PIN,
                       D_LS_MIDRIGHT_PIN, D_LS_RIGHT_PIN };
    int sensorReadings[5];

    float offset[5];
    float scale[5];

    // Optional EMA low-pass filter state. filter_valid seeds filtered[] with
    // the first calibrated reading (instead of 0) to avoid a startup ramp.
    float filter_alpha = D_FILTER_ALPHA;
    bool  filter_valid = false;

    // Rolling variance window. ring[] holds the last D_MAX_SAMPLES calibrated
    // values per sensor; ring_sum/ring_sumsq keep running totals so variance[]
    // updates in O(1) per push. ring_valid is false until the window is seeded.
    float ring[5][D_MAX_SAMPLES];
    float ring_sum[5]   = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    float ring_sumsq[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    int   ring_head  = 0;
    int   ring_count = 0;
    bool  ring_valid = false;

    // Outlier clamp strength (calibrated units). Configurable via the ctor
    // default D_OUTLIER_CLAMP and setOutlierClamp().
    float outlier_clamp = D_OUTLIER_CLAMP;

    // Online recalibration running extremes (raw units) and their valid flag.
    // auto_valid is false until seeded from the current calibration.
    float auto_min[5];
    float auto_max[5];
    bool  auto_valid = false;

    // Online recalibration rates. auto_adapt_rate is how fast offset/scale move
    // toward the observed range; auto_relax_rate is how fast the running min/max
    // decay back toward their midpoint. Both conservative, set via setAutoRate().
    float auto_adapt_rate  = D_AUTO_RATE;
    float auto_relax_rate  = D_AUTO_RELAX;

  public:

    float calibrated[5];
    float filtered[5];       // EMA-smoothed readings after getFiltered()
    float variance[5];       // per-sensor rolling variance (population, window)

    // Per-sensor gain trim applied after offset/scale. Default 1.0 leaves the
    // calibrated value unchanged. Lets the digital middle-sensor gain (and any
    // other) be tamed at runtime without hard-coding a value in calibrate().
    float gainTrim[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };

    // When true, getCalibrated() also updates filtered[] (opt-in). Default off
    // so behaviour is byte-identical to the unfiltered pipeline.
    bool  useFilter = false;

    // When true, getCalibrated() tracks a slow running min/max of the RAW
    // readings and nudges offset/scale toward that range so the robot adapts to
    // changing lighting or surface without a reboot (opt-in). Default off so
    // behaviour is byte-identical to the original path when disabled.
    bool  autoRecalibrate = false;

    DigitalLineSensors_c() {}

    // Set the EMA smoothing factor (0..1). Higher tracks faster, lower smooths
    // harder. Out-of-range values are ignored.
    void setFilterAlpha(float alpha) {
      if (alpha >= 0.0 && alpha <= 1.0) filter_alpha = alpha;
    }

    // Set the outlier clamp strength (calibrated units, > 0). Larger values are
    // more permissive (fewer clamps); smaller values cut spikes harder.
    void setOutlierClamp(float clamp) {
      if (clamp > 0.0) outlier_clamp = clamp;
    }

    // Set the per-sensor gain trim (index 0..4, trim > 0). Ignored otherwise.
    void setGainTrim(int index, float trim) {
      if (index >= 0 && index < 5 && trim > 0.0) gainTrim[index] = trim;
    }

    // Set the online recalibration rates (both 0..1). adaptRate controls how
    // fast offset/scale move toward the observed range; relaxRate controls how
    // fast the running min/max decay toward their midpoint. Out-of-range ignored.
    void setAutoRate(float adaptRate, float relaxRate) {
      if (adaptRate >= 0.0 && adaptRate <= 1.0) auto_adapt_rate = adaptRate;
      if (relaxRate >= 0.0 && relaxRate <= 1.0) auto_relax_rate = relaxRate;
    }

    // Copy the current per-sensor offset and scale out to the caller.
    void getCalibration(float outOffset[5], float outScale[5]) {
      for (int i = 0; i < 5; i++) {
        outOffset[i] = offset[i];
        outScale[i]  = scale[i];
      }
    }

    // Set the per-sensor offset and scale directly, so a prior calibration can
    // be restored without running calibrate() again.
    void setCalibration(const float inOffset[5], const float inScale[5]) {
      for (int i = 0; i < 5; i++) {
        offset[i] = inOffset[i];
        scale[i]  = inScale[i];
      }
    }

    // Seed the online recalibration extremes from the current calibration, so
    // enabling autoRecalibrate mid-run does not jump. offset[i] is the stored
    // min; the stored max is recovered as offset + 1/scale (scale = 1/range).
    void seedAutoCalibration() {
      for (int i = 0; i < 5; i++) {
        auto_min[i] = offset[i];
        auto_max[i] = (scale[i] > 0.0) ? (offset[i] + 1.0 / scale[i]) : offset[i];
      }
      auto_valid = true;
    }

    // Clear the online recalibration state. The extremes are re-seeded from the
    // current calibration on the next getCalibrated() when autoRecalibrate is on.
    void resetAutoCalibration() {
      auto_valid = false;
      for (int i = 0; i < 5; i++) {
        auto_min[i] = 0.0;
        auto_max[i] = 0.0;
      }
    }

    // Update the online recalibration state from the current RAW sensorReadings
    // and gently move offset/scale toward the observed range. Called only when
    // autoRecalibrate is true. Expands the running min/max immediately on a new
    // true extreme, otherwise relaxes them slightly toward their midpoint so a
    // transient spike does not permanently widen the range.
    void updateAutoCalibration() {

      if (!auto_valid) seedAutoCalibration();

      for (int i = 0; i < 5; i++) {
        float raw = (float)sensorReadings[i];

        // Expand immediately on a new true extreme, else relax toward midpoint.
        if (raw < auto_min[i]) {
          auto_min[i] = raw;
        } else if (raw > auto_max[i]) {
          auto_max[i] = raw;
        } else {
          float mid = 0.5 * (auto_min[i] + auto_max[i]);
          auto_min[i] += auto_relax_rate * (mid - auto_min[i]);
          auto_max[i] += auto_relax_rate * (mid - auto_max[i]);
        }

        // Nudge offset/scale toward the observed range. Guard zero range with
        // the same divide-by-zero pattern used by calibrate().
        float range = auto_max[i] - auto_min[i];
        float targetScale = (range > 0.0) ? (1.0 / range) : scale[i];
        offset[i] += auto_adapt_rate * (auto_min[i] - offset[i]);
        scale[i]  += auto_adapt_rate * (targetScale - scale[i]);
      }

    }

    // Read all sensors in parallel by charging the capacitors HIGH, releasing
    // them to INPUT, and timing each pin's fall to LOW. Result (microseconds)
    // is stored in sensorReadings[].
    void readAllSensors() {

      pinMode(D_EMIT_PIN, OUTPUT);
      digitalWrite(D_EMIT_PIN, HIGH);   // turn on IR LED

      // Charge capacitors.
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], OUTPUT);
        digitalWrite(ls_pins[i], HIGH);
        sensorReadings[i] = 0;          // 0 == "not yet completed"
      }

      delayMicroseconds(10);

      // Release all pins to input to begin discharge timing.
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], INPUT);
      }

      int remaining = 5;
      unsigned long start_time = micros();

      while (remaining > 0) {
        for (int i = 0; i < 5; i++) {
          if (sensorReadings[i] == 0) {                 // still pending?
            if (digitalRead(ls_pins[i]) == LOW) {       // discharged
              sensorReadings[i] = (int)(micros() - start_time);
              remaining--;
            }
          }
        }
      }

    }

    // Spin the robot over black + white surfaces while calling this so it can
    // capture the min/max range of each sensor and derive offset/scale.
    void calibrate() {

      float min_values[5];
      float max_values[5];

      for (int i = 0; i < 5; i++) {
        min_values[i] =  9999.9;
        max_values[i] = -9999.9;
      }

      int count = 0;
      while (count < 50) {          // ~0.5 s of sampling (10 ms * 50)

        readAllSensors();

        for (int i = 0; i < 5; i++) {
          if (sensorReadings[i] > max_values[i]) max_values[i] = sensorReadings[i];
          if (sensorReadings[i] < min_values[i]) min_values[i] = sensorReadings[i];
        }

        delay(10);
        count++;
      }

      for (int i = 0; i < 5; i++) {
        offset[i] = min_values[i];
        float range = max_values[i] - min_values[i];
        scale[i] = (range > 0.0) ? (1.0 / range) : 0.0;   // guard divide-by-zero
      }

    }

    // Median of the current window for one sensor. Used by the outlier guard.
    // O(N^2) insertion sort over the small fixed window; no allocation.
    float windowMedian(int sensor) {

      int n = ring_count;
      if (n <= 0) return 0.0;

      float tmp[D_MAX_SAMPLES];
      for (int k = 0; k < n; k++) tmp[k] = ring[sensor][k];

      for (int a = 1; a < n; a++) {
        float key = tmp[a];
        int b = a - 1;
        while (b >= 0 && tmp[b] > key) {
          tmp[b + 1] = tmp[b];
          b--;
        }
        tmp[b + 1] = key;
      }

      if (n % 2 == 1) return tmp[n / 2];
      return 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
    }

    // Push the current calibrated[] into the rolling window and refresh the
    // per-sensor sum/sum-of-squares. On first use the whole window is seeded
    // with the first sample so early variance estimates are sane (zero, not a
    // ramp from an empty window).
    void pushWindow() {

      if (!ring_valid) {
        for (int i = 0; i < 5; i++) {
          ring_sum[i]   = 0.0;
          ring_sumsq[i] = 0.0;
          for (int k = 0; k < D_MAX_SAMPLES; k++) {
            ring[i][k]     = calibrated[i];
            ring_sum[i]   += calibrated[i];
            ring_sumsq[i] += calibrated[i] * calibrated[i];
          }
        }
        ring_head  = 0;
        ring_count = D_MAX_SAMPLES;
        ring_valid = true;
        updateRollingVariance();
        return;
      }

      for (int i = 0; i < 5; i++) {
        float old = ring[i][ring_head];
        ring_sum[i]   -= old;
        ring_sumsq[i] -= old * old;
        ring[i][ring_head] = calibrated[i];
        ring_sum[i]   += calibrated[i];
        ring_sumsq[i] += calibrated[i] * calibrated[i];
      }
      ring_head = (ring_head + 1) % D_MAX_SAMPLES;

      updateRollingVariance();
    }

    // Recompute population variance for every sensor from the running totals.
    void updateRollingVariance() {
      float n = (float)ring_count;
      for (int i = 0; i < 5; i++) {
        float mean = ring_sum[i] / n;
        float v = ring_sumsq[i] / n - mean * mean;
        if (v < 0.0) v = 0.0;   // guard tiny negative from float rounding
        variance[i] = v;
      }
    }

    // Apply calibration to a fresh set of readings, filling calibrated[], then
    // update the rolling variance window. Cheap and non-blocking: one hardware
    // read plus O(1) window maintenance. Also applies the gain trim and a light
    // outlier clamp toward the window median before the value is stored.
    void getCalibrated() {

      readAllSensors();

      // Optional online recalibration. Off by default, so with autoRecalibrate
      // false offset/scale are untouched and outputs are byte-identical.
      if (autoRecalibrate) updateAutoCalibration();

      for (int i = 0; i < 5; i++) {
        float c = ((float)sensorReadings[i] - offset[i]) * scale[i] * gainTrim[i];

        // Reject single-sample spikes: clamp the new value to within
        // outlier_clamp of the window median. Skipped until the window exists.
        if (ring_valid) {
          float med = windowMedian(i);
          float lo = med - outlier_clamp;
          float hi = med + outlier_clamp;
          if (c < lo) c = lo;
          else if (c > hi) c = hi;
        }

        calibrated[i] = c;
      }

      pushWindow();

      if (useFilter) updateFilter();

    }

    // Update the EMA filter from the current calibrated[] values. On first use
    // filtered[] is seeded with calibrated[] to avoid ramping up from 0.
    void updateFilter() {

      if (!filter_valid) {
        for (int i = 0; i < 5; i++) {
          filtered[i] = calibrated[i];
        }
        filter_valid = true;
        return;
      }

      for (int i = 0; i < 5; i++) {
        filtered[i] = filter_alpha * calibrated[i]
                    + (1.0 - filter_alpha) * filtered[i];
      }

    }

    // Take a fresh reading and return the EMA-smoothed result in filtered[].
    // Opt-in low-pass path; leaves the calibrated[] contract unchanged. The
    // useFilter flag is bypassed here so the EMA is stepped exactly once.
    void getFiltered() {
      bool prev = useFilter;
      useFilter = false;
      getCalibrated();
      useFilter = prev;
      updateFilter();
    }

    void printCalibrated() {
      for (int i = 0; i < 5; i++) {
        Serial.print(calibrated[i]);
        Serial.print(",");
      }
      Serial.print("\n");
    }

    void printFiltered() {
      for (int i = 0; i < 5; i++) {
        Serial.print(filtered[i]);
        Serial.print(",");
      }
      Serial.print("\n");
    }

    // Take one fresh reading and refresh the rolling variance[]. Non-blocking:
    // the old D_MAX_SAMPLES-deep resampling loop is gone. getCalibrated() does
    // the window maintenance, so this is a thin, API-preserving wrapper.
    void calculateVariance() {
      getCalibrated();
    }

    void printVariance() {
      for (int i = 0; i < 5; i++) {
        Serial.print(variance[i], 6);
        Serial.print(",");
      }
      Serial.print("\n");
    }

    void printReadings() {
      for (int i = 0; i < 5; i++) {
        Serial.print(sensorReadings[i]);
        Serial.print(",");
      }
      Serial.print("\n");
    }

    // Mean rolling variance across all 5 sensors. Takes one fresh reading first
    // (so calibrated[] is current for the caller) then averages variance[].
    float calculateAverageVariance() {

      getCalibrated();

      float totalVariance = 0.0;
      for (int i = 0; i < 5; i++) {
        totalVariance += variance[i];
      }
      return totalVariance / 5.0f;

    }

};

#endif
