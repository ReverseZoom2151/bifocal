// Include guard: stops this header being pulled in more than once.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Size of the rolling theta history buffer.
#define THETA_LOG_SIZE 100

// Tracks the robot's pose (x, y, theta) by dead reckoning from the two wheel
// encoders (differential-drive odometry).
class Kinematics_c {

  private:

    long lastLeftEncoderCount  = 0;
    long lastRightEncoderCount = 0;

    // Pololu 3Pi+ geometry. These are nominal values -- calibrate for accuracy.
    // They are no longer const so setGeometry() can override them at runtime
    // after an empirical calibration (see docs/odometry-calibration.md). The
    // defaults below keep the original behaviour unchanged if nothing is set.
    float wheelRadius   = 16.5;   // wheel RADIUS in mm (~33 mm diameter)
    float wheelSeparation = 42.5; // distance between wheel contact points, mm
    float encoderCountsPerRevolution = 358.3;
    float mmPerCount = (2.0 * PI * wheelRadius) / encoderCountsPerRevolution;

    // Rolling log of recent theta values (circular buffer).
    float thetaLog[THETA_LOG_SIZE];

  public:

    float xPos = 0;   // global X position, mm
    float yPos = 0;   // global Y position, mm
    float theta = 0;  // heading, radians
    int currentIndex = 0;

    Kinematics_c() {}

    // Override the physical geometry with calibrated values and recompute the
    // derived mmPerCount factor. Call this once from setup() after measuring the
    // robot against ground truth (see docs/odometry-calibration.md). Passing the
    // nominal numbers reproduces the default behaviour exactly.
    void setGeometry(float newWheelRadius, float newWheelSeparation, float newCountsPerRev) {
      wheelRadius   = newWheelRadius;
      wheelSeparation = newWheelSeparation;
      encoderCountsPerRevolution = newCountsPerRev;
      mmPerCount = (2.0 * PI * wheelRadius) / encoderCountsPerRevolution;
    }

    // Accessors for the current geometry, in case a caller wants to read a value
    // back rather than assume the default.
    float getWheelRadius()     { return wheelRadius; }
    float getWheelSeparation() { return wheelSeparation; }
    float getCountsPerRev()    { return encoderCountsPerRevolution; }
    float getMmPerCount()      { return mmPerCount; }

    // Print the current geometry over serial as a labelled row, handy for
    // confirming which values the firmware is actually using.
    void printGeometry() {
      Serial.print("wheelRadius(mm)=");
      Serial.print(wheelRadius);
      Serial.print(" wheelSeparation(mm)=");
      Serial.print(wheelSeparation);
      Serial.print(" countsPerRev=");
      Serial.print(encoderCountsPerRevolution);
      Serial.print(" mmPerCount=");
      Serial.println(mmPerCount);
    }

    // Print the theta history collected so far (CSV row).
    void printTheta() {
      for (int i = 0; i < currentIndex; i++) {
        Serial.print(thetaLog[i]);
        Serial.print(",");
      }
    }

    // Integrate one odometry step from the latest encoder counts.
    void update(long leftEncoderCount, long rightEncoderCount) {

      long deltaLeft  = leftEncoderCount  - lastLeftEncoderCount;
      long deltaRight = rightEncoderCount - lastRightEncoderCount;

      lastLeftEncoderCount  = leftEncoderCount;
      lastRightEncoderCount = rightEncoderCount;

      float distanceLeft  = deltaLeft  * mmPerCount;
      float distanceRight = deltaRight * mmPerCount;
      float distanceAverage = (distanceLeft + distanceRight) / 2.0;

      xPos += distanceAverage * cos(theta);
      yPos += distanceAverage * sin(theta);

      // Standard differential-drive heading update: divide by the wheel
      // separation (the original code divided by 2 * separation, which
      // under-reported rotation by a factor of two).
      float deltaTheta = (distanceRight - distanceLeft) / wheelSeparation;
      theta += deltaTheta;

      // Store into circular buffer, bounded by THETA_LOG_SIZE.
      thetaLog[currentIndex] = theta;
      currentIndex++;
      if (currentIndex >= THETA_LOG_SIZE) {
        currentIndex = 0;
      }

    }

};

#endif
