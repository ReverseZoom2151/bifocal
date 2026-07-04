#ifndef KALMAN_H
#define KALMAN_H

// 1D Kalman line-position estimator for Bifocal.
//
// The line follower already computes two noisy measurements of the same
// quantity, the line position in [-1, 1]:
//   - e_a: the analog-array line position, with live average variance g_var_a.
//   - e_d: the digital-array line position, with live average variance g_var_d.
//
// METHOD_FUSION in line_following.ino blends these by inverse-variance
// weighting: it trusts whichever array is currently steadier. That blend is
// exactly the Bayesian combination of two Gaussian measurements, and this class
// makes that reframing explicit as a scalar Kalman filter.
//
// The key connection: each array's live variance IS its measurement noise R.
// A steady array (small variance) is a trusted measurement (small R), so its
// Kalman gain is large and it pulls the estimate hard; a noisy array (large
// variance, large R) barely moves the estimate. Fusing analog then digital in
// sequence, with R set to each array's live variance, reproduces the
// inverse-variance blend while also carrying an estimate covariance P forward
// in time (the predict step), which plain per-tick blending does not.
//
// State:
//   x  estimated line position (scalar)
//   P  estimate covariance (uncertainty in x)
//
// AVR-safe: all float, no dynamic allocation, no STL, no VLAs. Does not need
// Arduino.h, so it also compiles standalone on the host for tests.

class KalmanLine1D {

  public:

    float x;   // current estimate of the true line position
    float P;   // current estimate covariance
    float Q;   // process noise added each predict() (random-walk model)
    float eps; // guard added to R so a zero variance cannot divide by zero

    // Defaults: start centred, moderate initial uncertainty, small process
    // noise, and a tiny R guard matching the project's FUSION_EPS in spirit.
    KalmanLine1D() {
      x   = 0.0;
      P   = 1.0;
      Q   = 0.0001;
      eps = 0.0001;
    }

    // Reset to a chosen initial estimate and covariance.
    void reset(float x0 = 0.0, float P0 = 1.0) {
      x = x0;
      P = P0;
    }

    // Random-walk prediction. The true line position can drift between ticks,
    // so the state is unchanged but its uncertainty grows by the process noise.
    void predict() {
      P += Q;
    }

    // Standard scalar Kalman measurement update.
    //   R = measurementVariance (the array's live variance), guarded by eps.
    //   K = P / (P + R)
    //   x += K * (measurement - x)
    //   P  = (1 - K) * P
    void update(float measurement, float measurementVariance) {
      float R = measurementVariance + eps;   // eps keeps R > 0 when variance = 0
      float K = P / (P + R);                  // Kalman gain in [0, 1]
      x += K * (measurement - x);
      P  = (1.0 - K) * P;
    }

    // Convenience one-shot fusion of the two arrays for a control tick. Predicts
    // once, then applies the analog and digital measurements in sequence with
    // each array's live variance as its measurement noise. Returns the fused
    // estimate. This is the Bayesian form of inverse-variance blending.
    float fuse(float eAnalog, float varAnalog,
               float eDigital, float varDigital) {
      predict();
      update(eAnalog, varAnalog);
      update(eDigital, varDigital);
      return x;
    }

};

#endif
