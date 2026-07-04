#ifndef ANALOGLINESENSORS_H
#define ANALOGLINESENSORS_H

// Number of samples averaged per reading and used per variance estimate. This
// is also the length of the rolling variance window.
#define A_MAX_SAMPLES 10

// Default smoothing factor for the optional EMA low-pass filter on calibrated
// readings. Range 0..1: higher tracks faster, lower smooths harder.
#define A_FILTER_ALPHA 0.4

// Default outlier clamp for the calibrated path. A new per-sensor calibrated
// value that deviates from the window median by more than this (in calibrated
// 0..1 units) is clamped back toward the median before it reaches variance[].
// Mild by default so normal motion is untouched and only large spikes are cut.
#define A_OUTLIER_CLAMP 0.3

// IR emitter and the 5 analog line-sensor pins (left -> right).
#define A_EMIT_PIN      11
#define A_LS_LEFT_PIN     A11
#define A_LS_MIDLEFT_PIN  A0
#define A_LS_MIDDLE_PIN   A2
#define A_LS_MIDRIGHT_PIN A3
#define A_LS_RIGHT_PIN    A4

// Reads the 5-element IR array in analog mode. Raw analogRead() values are
// normalised to a 0..1 range using per-sensor offset/scale captured during
// calibrate(). Provides a per-sensor variance estimate used by the
// variance-based switching logic.
//
// Variance is now computed with a rolling (non-blocking) window: getCalibrated()
// pushes the newest calibrated[] into a per-sensor ring buffer and updates
// variance[] in O(1) from running sum and sum-of-squares. This replaces the old
// per-call blocking loop that took A_MAX_SAMPLES fresh readings each tick, so
// the controller no longer stalls. It is the one intended behavioural change;
// all other features default to a no-op.
class AnalogLineSensors_c {

  private:

    // Pin numbers for convenient indexed access (index 0 = left ... 4 = right).
    int ls_pins[5] = { A_LS_LEFT_PIN, A_LS_MIDLEFT_PIN, A_LS_MIDDLE_PIN,
                       A_LS_MIDRIGHT_PIN, A_LS_RIGHT_PIN };
    int sensorReadings[5];   // most recent (averaged) raw readings

    // Per-sensor calibration, applied to every future reading.
    float offset[5];
    float scale[5];

    // Optional EMA low-pass filter state. filter_valid seeds filtered[] with
    // the first calibrated reading (instead of 0) to avoid a startup ramp.
    float filter_alpha = A_FILTER_ALPHA;
    bool  filter_valid = false;

    // Rolling variance window. ring[] holds the last A_MAX_SAMPLES calibrated
    // values per sensor; ring_sum/ring_sumsq keep running totals so variance[]
    // updates in O(1) per push. ring_valid is false until the window is seeded.
    float ring[5][A_MAX_SAMPLES];
    float ring_sum[5]   = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    float ring_sumsq[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    int   ring_head  = 0;
    int   ring_count = 0;
    bool  ring_valid = false;

    // Outlier clamp strength (calibrated units). Configurable via the ctor
    // default A_OUTLIER_CLAMP and setOutlierClamp().
    float outlier_clamp = A_OUTLIER_CLAMP;

  public:

    float calibrated[5];     // normalised readings (~0..1) after getCalibrated()
    float filtered[5];       // EMA-smoothed readings after getFiltered()
    float variance[5];       // per-sensor rolling variance (population, window)

    // Per-sensor gain trim applied after offset/scale. Default 1.0 leaves the
    // calibrated value unchanged. Lets the digital middle-sensor gain (and any
    // other) be tamed at runtime without hard-coding a value in calibrate().
    float gainTrim[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };

    // When true, getCalibrated() also updates filtered[] (opt-in). Default off
    // so behaviour is byte-identical to the unfiltered pipeline.
    bool  useFilter = false;

    AnalogLineSensors_c() {}

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

    // Configure the emitter and sensor pins. Cheap, safe to call repeatedly.
    void setupAllLineSensors() {

      pinMode(A_EMIT_PIN, OUTPUT);
      digitalWrite(A_EMIT_PIN, HIGH);   // turn on IR LEDs

      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], INPUT_PULLUP);
      }

    }

    // Read every sensor once, averaging A_MAX_SAMPLES analog samples per sensor
    // to reduce noise. Result is stored in sensorReadings[].
    void readAllSensors() {

      setupAllLineSensors();

      long acc[5] = { 0, 0, 0, 0, 0 };

      for (int s = 0; s < A_MAX_SAMPLES; s++) {
        for (int sensor = 0; sensor < 5; sensor++) {
          acc[sensor] += analogRead(ls_pins[sensor]);
        }
      }

      for (int sensor = 0; sensor < 5; sensor++) {
        sensorReadings[sensor] = (int)(acc[sensor] / A_MAX_SAMPLES);
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

      // Derive calibration: offset removes the minimum, scale maps range -> 1.0.
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

      float tmp[A_MAX_SAMPLES];
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
          for (int k = 0; k < A_MAX_SAMPLES; k++) {
            ring[i][k]     = calibrated[i];
            ring_sum[i]   += calibrated[i];
            ring_sumsq[i] += calibrated[i] * calibrated[i];
          }
        }
        ring_head  = 0;
        ring_count = A_MAX_SAMPLES;
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
      ring_head = (ring_head + 1) % A_MAX_SAMPLES;

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
    // the old A_MAX_SAMPLES-deep resampling loop is gone. getCalibrated() does
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
    // NOTE: kept identical in form to the digital version so the two arrays are
    // compared on the same basis by the switching logic.
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
