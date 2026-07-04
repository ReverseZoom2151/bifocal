#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#include <EEPROM.h>

// EEPROM calibration persistence.
// ------------------------------------------------------------------
// Stores both sensor arrays' calibration (analog and digital), each as an
// offset[5] and scale[5] pair (10 floats per array, 20 floats total). A small
// magic + version header guards the block so a blank or stale EEPROM is never
// loaded as if it were valid calibration.
//
// The layout is a single fixed struct written at EEPROM address 0 via
// EEPROM.put()/EEPROM.get(). No dynamic allocation, no String; AVR-safe.
//
// Typical use from the sketch:
//   float aOff[5], aScale[5], dOff[5], dScale[5];
//   a_sensors.getCalibration(aOff, aScale);
//   d_sensors.getCalibration(dOff, dScale);
//   saveCalibration(aOff, aScale, dOff, dScale);
// and on load:
//   if (loadCalibration(aOff, aScale, dOff, dScale)) {
//     a_sensors.setCalibration(aOff, aScale);
//     d_sensors.setCalibration(dOff, dScale);
//   }

// Magic marks a valid block; bump PERSIST_VERSION if the layout changes so old
// data is rejected instead of misread.
#define PERSIST_MAGIC    0x42494643UL   // 'BIFC'
#define PERSIST_VERSION  1
#define PERSIST_ADDR     0              // EEPROM start address of the block

// One contiguous record. Keep field order stable across versions.
struct CalibrationBlob {
  unsigned long magic;
  unsigned int  version;
  float aOffset[5];
  float aScale[5];
  float dOffset[5];
  float dScale[5];
};

// Copy 5 floats. Small helper to keep the load/save code readable.
inline void persistCopy5(float dst[5], const float src[5]) {
  for (int i = 0; i < 5; i++) dst[i] = src[i];
}

// Return true if a valid calibration block is present in EEPROM.
inline bool hasValidCalibration() {
  CalibrationBlob blob;
  EEPROM.get(PERSIST_ADDR, blob);
  return (blob.magic == PERSIST_MAGIC && blob.version == PERSIST_VERSION);
}

// Write the four calibration arrays to EEPROM with a valid header. Uses
// EEPROM.put() which only rewrites bytes that changed, sparing the cells.
inline void saveCalibration(const float aOff[5], const float aScale[5],
                            const float dOff[5], const float dScale[5]) {
  CalibrationBlob blob;
  blob.magic   = PERSIST_MAGIC;
  blob.version = PERSIST_VERSION;
  persistCopy5(blob.aOffset, aOff);
  persistCopy5(blob.aScale,  aScale);
  persistCopy5(blob.dOffset, dOff);
  persistCopy5(blob.dScale,  dScale);
  EEPROM.put(PERSIST_ADDR, blob);
}

// Load calibration into the four output arrays. Returns false (and leaves the
// outputs untouched) when no valid block is stored, so blank EEPROM never
// yields garbage calibration.
inline bool loadCalibration(float aOff[5], float aScale[5],
                            float dOff[5], float dScale[5]) {
  CalibrationBlob blob;
  EEPROM.get(PERSIST_ADDR, blob);
  if (blob.magic != PERSIST_MAGIC || blob.version != PERSIST_VERSION) {
    return false;
  }
  persistCopy5(aOff,   blob.aOffset);
  persistCopy5(aScale, blob.aScale);
  persistCopy5(dOff,   blob.dOffset);
  persistCopy5(dScale, blob.dScale);
  return true;
}

// Invalidate any stored calibration by clearing the magic. A following
// hasValidCalibration()/loadCalibration() will report no valid data.
inline void clearCalibration() {
  unsigned long badMagic = 0;
  EEPROM.put(PERSIST_ADDR, badMagic);
}

#endif
