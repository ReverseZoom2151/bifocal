#ifndef DIGITALLINESENSORS_H
#define DIGITALLINESENSORS_H

// Number of samples used per variance estimate.
#define D_MAX_SAMPLES 10

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
class DigitalLineSensors_c {

  private:

    int ls_pins[5] = { D_LS_LEFT_PIN, D_LS_MIDLEFT_PIN, D_LS_MIDDLE_PIN,
                       D_LS_MIDRIGHT_PIN, D_LS_RIGHT_PIN };
    int sensorReadings[5];

    float offset[5];
    float scale[5];

  public:

    float calibrated[5];
    float variance[5];

    DigitalLineSensors_c() {}

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

    // Apply calibration to a fresh set of readings, filling calibrated[].
    void getCalibrated() {

      readAllSensors();

      for (int i = 0; i < 5; i++) {
        calibrated[i] = ((float)sensorReadings[i] - offset[i]) * scale[i];
      }

    }

    void printCalibrated() {
      for (int i = 0; i < 5; i++) {
        Serial.print(calibrated[i]);
        Serial.print(",");
      }
      Serial.print("\n");
    }

    // Estimate per-sensor variance over D_MAX_SAMPLES calibrated readings and
    // store it in variance[]. Lower variance == more stable sensor data.
    void calculateVariance() {

      const int num_samples = D_MAX_SAMPLES;
      float samples[5][D_MAX_SAMPLES];         // fixed size (no VLA)
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
