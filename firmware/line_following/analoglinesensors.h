#ifndef ANALOGLINESENSORS_H
#define ANALOGLINESENSORS_H

// Number of samples averaged per reading and used per variance estimate.
#define A_MAX_SAMPLES 10

// Default smoothing factor for the optional EMA low-pass filter on calibrated
// readings. Range 0..1: higher tracks faster, lower smooths harder.
#define A_FILTER_ALPHA 0.4

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

  public:

    float calibrated[5];     // normalised readings (~0..1) after getCalibrated()
    float filtered[5];       // EMA-smoothed readings after getFiltered()
    float variance[5];       // per-sensor variance after calculateVariance()

    // When true, getCalibrated() also updates filtered[] (opt-in). Default off
    // so behaviour is byte-identical to the unfiltered pipeline.
    bool  useFilter = false;

    AnalogLineSensors_c() {}

    // Set the EMA smoothing factor (0..1). Higher tracks faster, lower smooths
    // harder. Out-of-range values are ignored.
    void setFilterAlpha(float alpha) {
      if (alpha >= 0.0 && alpha <= 1.0) filter_alpha = alpha;
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

    // Apply calibration to a fresh set of readings, filling calibrated[].
    void getCalibrated() {

      readAllSensors();

      for (int i = 0; i < 5; i++) {
        calibrated[i] = ((float)sensorReadings[i] - offset[i]) * scale[i];
      }

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

    // Estimate per-sensor variance over A_MAX_SAMPLES calibrated readings and
    // store it in variance[]. Lower variance == more stable sensor data.
    void calculateVariance() {

      const int num_samples = A_MAX_SAMPLES;
      float samples[5][A_MAX_SAMPLES];         // fixed size (no VLA)
      float sum[5]  = { 0.0, 0.0, 0.0, 0.0, 0.0 };
      float mean[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

      for (int sample = 0; sample < num_samples; sample++) {
        getCalibrated();
        for (int sensor = 0; sensor < 5; sensor++) {
          samples[sensor][sample] = calibrated[sensor];
          sum[sensor] += calibrated[sensor];
        }
      }

      for (int sensor = 0; sensor < 5; sensor++) {
        mean[sensor] = sum[sensor] / (float)num_samples;
      }

      for (int sensor = 0; sensor < 5; sensor++) {
        variance[sensor] = 0.0;
        for (int sample = 0; sample < num_samples; sample++) {
          float d = samples[sensor][sample] - mean[sensor];
          variance[sensor] += d * d;
        }
        variance[sensor] /= (float)num_samples;
      }

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

    // Mean variance across all 5 sensors. Refreshes variance[] first.
    // NOTE: kept identical in form to the digital version so the two arrays are
    // compared on the same basis by the switching logic.
    float calculateAverageVariance() {

      calculateVariance();

      float totalVariance = 0.0;
      for (int i = 0; i < 5; i++) {
        totalVariance += variance[i];
      }
      return totalVariance / 5.0f;

    }

};

#endif
