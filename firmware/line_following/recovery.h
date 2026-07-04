// Include guard: stops this header being pulled in more than once.
#ifndef _RECOVERY_H
#define _RECOVERY_H

// Line-loss recovery and gap tolerance for the Bifocal line follower.
// ------------------------------------------------------------------
// Today the controller simply stops the motors when the steering sensors lose
// the line (a_sum/d_sum drop below ANALOG_STOP_SUM/DIGITAL_STOP_SUM in
// line_following.ino). This header adds a small, self-contained state machine
// that instead tries to keep going: it coasts across small gaps using
// odometry, then sweeps toward the last-known line side to reacquire, and only
// gives up (stops) after a configurable search limit.
//
// This header is deliberate about NOT depending on Arduino.h. It never calls
// millis(); the caller passes elapsed time in as a parameter. It uses only
// int/float and has no dynamic allocation or STL, so it is AVR-safe and also
// compiles and runs on a normal PC for host-side testing.
//
// How the returned action maps to motor commands
// -----------------------------------------------
// update() returns a RecoveryAction with three fields:
//   state       - the current RecoveryState (for logging / decisions)
//   useNormalSteering - true when the caller's own steering term W should be
//                       used unchanged (FOLLOWING); false when the caller must
//                       override steering with turnBias (COASTING/SEARCHING/
//                       STOPPED)
//   turnBias    - a turn demand in [-1, 1], same convention as the steering
//                 term W in line_following.ino (positive turns one way,
//                 negative the other). Only meaningful when
//                 useNormalSteering is false.
//   speedScale  - a forward-speed multiplier in [0, 1]. Multiply the normal
//                 forward bias by this. 0 means stop.
//
// Suggested wiring in followLine() (NOT done here, this header is standalone):
//   RecoveryAction act = recovery.update(linePresent, g_line_error,
//                                        distance_mm, millis() - t0);
//   float W = act.useNormalSteering ? computeControl() : act.turnBias;
//   applySteering(W, act.speedScale);   // scale the forward bias by speedScale
//
// State meanings
// --------------
//   FOLLOWING - line is present, follow normally (use the caller's steering).
//   COASTING  - line just lost. Keep going roughly straight (turnBias 0) at
//               near-normal speed to bridge a small gap, for up to a
//               configurable distance of travel.
//   SEARCHING - still lost after coasting. Sweep/arc toward the last-known line
//               side (sign of the last steering error) at reduced speed, for up
//               to a configurable amount of time.
//   STOPPED   - gave up after the search limit. Motors held at zero.

// ---- Default thresholds (override with the setters below) ------------------

// How far (in the same distance units the caller passes, e.g. mm) the robot may
// coast straight after losing the line before it starts actively searching.
#ifndef RECOVERY_MAX_COAST_DIST
#define RECOVERY_MAX_COAST_DIST 40.0f
#endif

// How long (in the same time units the caller passes, e.g. ms) the robot may
// search before giving up and stopping.
#ifndef RECOVERY_MAX_SEARCH_TIME
#define RECOVERY_MAX_SEARCH_TIME 1500.0f
#endif

// Turn demand applied while searching, in [0, 1]. It is multiplied by the sign
// of the last-known line side to sweep the correct way.
#ifndef RECOVERY_SEARCH_TURN
#define RECOVERY_SEARCH_TURN 0.6f
#endif

// Forward-speed multiplier while coasting. Kept high so the robot carries its
// momentum straight across the gap.
#ifndef RECOVERY_COAST_SPEED
#define RECOVERY_COAST_SPEED 1.0f
#endif

// Forward-speed multiplier while searching. Reduced so the sweep is controlled.
#ifndef RECOVERY_SEARCH_SPEED
#define RECOVERY_SEARCH_SPEED 0.5f
#endif

// Deadband on the steering error below which we do not update the remembered
// line side (avoids latching a side from sensor noise near centre).
#ifndef RECOVERY_SIDE_DEADBAND
#define RECOVERY_SIDE_DEADBAND 0.02f
#endif

// The recovery states.
enum RecoveryState {
  REC_FOLLOWING = 0,
  REC_COASTING  = 1,
  REC_SEARCHING = 2,
  REC_STOPPED   = 3
};

// The action the controller should apply this tick. See the header comment for
// how each field maps to motor commands.
struct RecoveryAction {
  RecoveryState state;
  bool  useNormalSteering;  // true: use caller's steering term; false: use turnBias
  float turnBias;           // [-1, 1], only used when useNormalSteering is false
  float speedScale;         // [0, 1] forward-bias multiplier
};

class LineRecovery_c {

  public:

    LineRecovery_c() {
      // Initialise tunables to the compile-time defaults, then reset state.
      maxCoastDist  = RECOVERY_MAX_COAST_DIST;
      maxSearchTime = RECOVERY_MAX_SEARCH_TIME;
      searchTurn    = RECOVERY_SEARCH_TURN;
      coastSpeed    = RECOVERY_COAST_SPEED;
      searchSpeed   = RECOVERY_SEARCH_SPEED;
      reset();
    }

    // Return to the normal following state and clear all recovery bookkeeping.
    void reset() {
      state       = REC_FOLLOWING;
      lastSide    = 0;      // -1 left, 0 unknown, +1 right (sign of last error)
      coastRefDist = 0.0f;  // distance mark taken when coasting began
      searchRefTime = 0.0f; // time mark taken when searching began
    }

    // ---- Tunable setters (all sane-defaulted in the constructor) -----------
    void setMaxCoastDist(float d)  { maxCoastDist  = d; }
    void setMaxSearchTime(float t) { maxSearchTime = t; }
    void setSearchTurn(float s)    { searchTurn    = s; }
    void setCoastSpeed(float s)    { coastSpeed    = s; }
    void setSearchSpeed(float s)   { searchSpeed   = s; }

    // ---- Read-back accessors ----------------------------------------------
    RecoveryState getState() const { return state; }
    int  getLastSide() const       { return lastSide; }
    float getMaxCoastDist() const  { return maxCoastDist; }
    float getMaxSearchTime() const { return maxSearchTime; }

    // Advance the state machine one tick and return the recommended action.
    //
    //   linePresent       - true if the steering sensors currently see the line
    //   lastError         - the latest steering error / turn term, [-1, 1]. Its
    //                        sign records which side the line was last on.
    //   distanceTravelled - a monotonically increasing cumulative distance
    //                        (e.g. mm from odometry). Used to bound coasting.
    //   elapsedMs         - a monotonically increasing elapsed time (e.g. ms).
    //                        Used to bound searching.
    RecoveryAction update(bool linePresent, float lastError,
                          float distanceTravelled, float elapsedMs) {

      // While the line is visible, keep the remembered side up to date so that,
      // once it is lost, SEARCHING sweeps back toward where it was last seen.
      if (linePresent) {
        if (lastError > RECOVERY_SIDE_DEADBAND) {
          lastSide = 1;
        } else if (lastError < -RECOVERY_SIDE_DEADBAND) {
          lastSide = -1;
        }
        // Line seen from any state means reacquired: resume normal following.
        state = REC_FOLLOWING;
        return makeAction();
      }

      // Line is not present. Drive the loss-recovery progression.
      switch (state) {

        case REC_FOLLOWING:
          // Just lost the line: begin coasting and mark where we were. Fall
          // through to the coast check so a zero coast distance escalates to
          // searching on the same tick.
          state = REC_COASTING;
          coastRefDist = distanceTravelled;
          // fallthrough

        case REC_COASTING:
          // Coast straight until we have travelled the allowed gap distance,
          // then escalate to an active search.
          if (distanceTravelled - coastRefDist >= maxCoastDist) {
            state = REC_SEARCHING;
            searchRefTime = elapsedMs;
          }
          break;

        case REC_SEARCHING:
          // Sweep toward the last-known side until the search time runs out.
          if (elapsedMs - searchRefTime >= maxSearchTime) {
            state = REC_STOPPED;
          }
          break;

        case REC_STOPPED:
        default:
          // Stay stopped until reset() or the line is seen again.
          break;
      }

      return makeAction();
    }

  private:

    RecoveryState state;
    int   lastSide;
    float coastRefDist;
    float searchRefTime;

    float maxCoastDist;
    float maxSearchTime;
    float searchTurn;
    float coastSpeed;
    float searchSpeed;

    // Build the action that corresponds to the current state.
    RecoveryAction makeAction() const {
      RecoveryAction a;
      a.state = state;

      switch (state) {
        case REC_FOLLOWING:
          a.useNormalSteering = true;
          a.turnBias   = 0.0f;
          a.speedScale = 1.0f;
          break;

        case REC_COASTING:
          // Straight ahead, near-normal speed, to carry across a small gap.
          a.useNormalSteering = false;
          a.turnBias   = 0.0f;
          a.speedScale = coastSpeed;
          break;

        case REC_SEARCHING:
          // Arc toward the last-known side. If the side is still unknown
          // (never had a clear error), sweep one default way (+).
          a.useNormalSteering = false;
          a.turnBias   = (lastSide >= 0 ? 1.0f : -1.0f) * searchTurn;
          a.speedScale = searchSpeed;
          break;

        case REC_STOPPED:
        default:
          a.useNormalSteering = false;
          a.turnBias   = 0.0f;
          a.speedScale = 0.0f;
          break;
      }
      return a;
    }

};

#endif
