#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

// Host-side stub of the small subset of the Arduino API that the Bifocal
// firmware headers reference. It lets the pure algorithmic classes compile and
// run on a normal PC (g++) with no real hardware. Sensor input functions
// (analogRead / digitalRead) are injectable so a test can preload the exact
// values those calls return and then assert on the resulting outputs.

#include <stdint.h>
#include <stdio.h>
#include <math.h>

// Arduino exposes PI as a macro.
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino integer typedefs used by the firmware.
typedef uint8_t byte;
typedef bool boolean;

// Pin mode and level constants.
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2
#define LOW          0
#define HIGH         1

// Analog pin macros. On real hardware these are opaque pin identifiers; here
// they just need to be distinct integers so indexed access works.
#define A0  14
#define A2  16
#define A3  17
#define A4  18
#define A11 25

// Arduino provides abs() and constrain() as macros.
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
#ifndef constrain
#define constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Injectable sensor read hooks.
// The functions receive the pin and a monotonically increasing call index so a
// test can produce deterministic value schedules (for example a value that is
// held constant across one readAllSensors block and stepped between blocks).
typedef int (*AnalogReadFn)(int pin, unsigned long index);
typedef int (*DigitalReadFn)(int pin, unsigned long index);

// Meyers-style accessors keep the mutable state header-only without needing a
// separate translation unit or C++17 inline variables.
inline AnalogReadFn& analogReadHook() { static AnalogReadFn f = 0; return f; }
inline DigitalReadFn& digitalReadHook() { static DigitalReadFn f = 0; return f; }
inline unsigned long& analogReadCounter() { static unsigned long c = 0; return c; }
inline unsigned long& digitalReadCounter() { static unsigned long c = 0; return c; }

// Test helpers to install a hook and reset the call counter.
inline void setAnalogReadFn(AnalogReadFn f) { analogReadHook() = f; }
inline void setDigitalReadFn(DigitalReadFn f) { digitalReadHook() = f; }
inline void resetAnalogRead() { analogReadCounter() = 0; }
inline void resetDigitalRead() { digitalReadCounter() = 0; }

// Digital I/O.
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void analogWrite(int, int) {}

inline int digitalRead(int pin) {
  unsigned long i = digitalReadCounter()++;
  if (digitalReadHook()) return digitalReadHook()(pin, i);
  return LOW;
}

inline int analogRead(int pin) {
  unsigned long i = analogReadCounter()++;
  if (analogReadHook()) return analogReadHook()(pin, i);
  return 0;
}

// Timing. Return a simple monotonically increasing microsecond count so any
// code that times an interval sees forward progress and never hangs.
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}

inline unsigned long micros() {
  static unsigned long t = 0;
  t += 1;
  return t;
}

inline unsigned long millis() {
  return micros() / 1000UL;
}

// Minimal Serial stub. The firmware print helpers are compiled as part of the
// class definitions even when a test never calls them, so every overload they
// use must exist. All output is discarded.
struct SerialStub {
  void begin(long) {}
  void begin(unsigned long) {}

  void print(char) {}
  void print(const char*) {}
  void print(int) {}
  void print(unsigned int) {}
  void print(long) {}
  void print(unsigned long) {}
  void print(double) {}
  void print(double, int) {}

  void println() {}
  void println(char) {}
  void println(const char*) {}
  void println(int) {}
  void println(unsigned int) {}
  void println(long) {}
  void println(unsigned long) {}
  void println(double) {}
  void println(double, int) {}
};

static SerialStub Serial;

#endif
