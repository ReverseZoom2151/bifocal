// Host-side tests for LineRecovery_c, the line-loss recovery state machine.
//
// The machine progresses FOLLOWING -> COASTING -> SEARCHING -> STOPPED while the
// line stays lost, and jumps straight back to FOLLOWING as soon as the line is
// seen again. update() takes the current line-present flag, the last steering
// error (whose sign records the last-known line side), a cumulative distance
// (bounds coasting) and an elapsed time (bounds searching), and returns a
// RecoveryAction with a state, a turn bias and a forward-speed scale.
//
// recovery.h has no Arduino dependency, so it is included directly with no stub.

#include "recovery.h"
#include "test_util.h"

// Line present should always report FOLLOWING and hand steering back to the
// caller at full speed.
static void test_present_is_following() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);

  RecoveryAction a = r.update(true, 0.3f, 0.0f, 0.0f);
  EXPECT(a.state == REC_FOLLOWING, "line present -> FOLLOWING");
  EXPECT(a.useNormalSteering, "FOLLOWING uses the caller's steering");
  EXPECT_NEAR(a.speedScale, 1.0f, 1e-6, "FOLLOWING runs at full speed");
}

// Losing the line briefly should COAST and keep moving (non-zero speed, roughly
// straight), so a small gap is bridged without stopping.
static void test_brief_loss_coasts() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);

  // Establish a following state and a known side (line to the right, error +).
  r.update(true, 0.5f, 0.0f, 0.0f);

  // Line disappears. Distance has only advanced a little (< max coast dist).
  RecoveryAction a = r.update(false, 0.5f, 10.0f, 100.0f);
  EXPECT(a.state == REC_COASTING, "brief loss -> COASTING");
  EXPECT(a.speedScale > 0.0f, "COASTING keeps the robot moving");
  EXPECT(!a.useNormalSteering, "COASTING overrides steering");
  EXPECT_NEAR(a.turnBias, 0.0f, 1e-6, "COASTING goes roughly straight");

  // A little further, still under the coast limit: still coasting.
  RecoveryAction b = r.update(false, 0.5f, 30.0f, 300.0f);
  EXPECT(b.state == REC_COASTING, "still under coast limit -> still COASTING");
}

// Once the coast distance limit is exceeded the machine should SEARCH, sweeping
// toward the last-known side. Here the last error was positive, so the sweep
// bias must be positive.
static void test_coast_limit_searches_right() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);
  r.setSearchTurn(0.6f);

  r.update(true, 0.5f, 0.0f, 0.0f);     // following, line on the right (+)
  r.update(false, 0.5f, 10.0f, 100.0f); // coasting begins at dist 10

  // Travel past 10 + 40 = 50 mm of cumulative distance: escalate to searching.
  RecoveryAction a = r.update(false, 0.5f, 55.0f, 500.0f);
  EXPECT(a.state == REC_SEARCHING, "past coast limit -> SEARCHING");
  EXPECT(a.turnBias > 0.0f, "positive last error -> sweep toward + side");
  EXPECT(!a.useNormalSteering, "SEARCHING overrides steering");
  EXPECT(a.speedScale > 0.0f && a.speedScale < 1.0f,
         "SEARCHING moves at reduced speed");
  EXPECT(r.getLastSide() == 1, "remembered side is right (+1)");
}

// The sweep direction must follow the sign of the last error: a negative last
// error (line on the left) yields a negative search bias.
static void test_search_sweeps_left_for_negative_error() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);
  r.setSearchTurn(0.6f);

  r.update(true, -0.5f, 0.0f, 0.0f);      // following, line on the left (-)
  r.update(false, -0.5f, 10.0f, 100.0f);  // coasting
  RecoveryAction a = r.update(false, -0.5f, 55.0f, 500.0f);
  EXPECT(a.state == REC_SEARCHING, "past coast limit -> SEARCHING");
  EXPECT(a.turnBias < 0.0f, "negative last error -> sweep toward - side");
  EXPECT(r.getLastSide() == -1, "remembered side is left (-1)");
}

// Searching past the time limit should give up and STOP (zero speed).
static void test_search_limit_stops() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);

  r.update(true, 0.5f, 0.0f, 0.0f);
  r.update(false, 0.5f, 10.0f, 100.0f);      // coasting
  RecoveryAction s = r.update(false, 0.5f, 55.0f, 500.0f); // searching @ t=500
  EXPECT(s.state == REC_SEARCHING, "searching begins");

  // Still within the search window (500 + 1500 = 2000): still searching.
  RecoveryAction mid = r.update(false, 0.5f, 60.0f, 1800.0f);
  EXPECT(mid.state == REC_SEARCHING, "within search window -> still SEARCHING");

  // Past the search window: give up.
  RecoveryAction a = r.update(false, 0.5f, 60.0f, 2100.0f);
  EXPECT(a.state == REC_STOPPED, "past search limit -> STOPPED");
  EXPECT_NEAR(a.speedScale, 0.0f, 1e-6, "STOPPED holds motors at zero");

  // Stays stopped on subsequent lost ticks.
  RecoveryAction b = r.update(false, 0.5f, 60.0f, 3000.0f);
  EXPECT(b.state == REC_STOPPED, "STOPPED is sticky while line stays lost");
}

// Reacquiring the line mid-search should snap straight back to FOLLOWING.
static void test_reacquire_mid_search() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);

  r.update(true, 0.5f, 0.0f, 0.0f);
  r.update(false, 0.5f, 10.0f, 100.0f);                 // coasting
  RecoveryAction s = r.update(false, 0.5f, 55.0f, 500.0f); // searching
  EXPECT(s.state == REC_SEARCHING, "searching before reacquisition");

  RecoveryAction a = r.update(true, 0.4f, 60.0f, 700.0f);  // line found again
  EXPECT(a.state == REC_FOLLOWING, "line reacquired mid-search -> FOLLOWING");
  EXPECT(a.useNormalSteering, "back to caller steering after reacquisition");
  EXPECT_NEAR(a.speedScale, 1.0f, 1e-6, "full speed restored after reacquire");
}

// Even from STOPPED, seeing the line again should recover to FOLLOWING (e.g. the
// robot was nudged back onto the line, or reset conditions changed).
static void test_reacquire_from_stopped() {
  LineRecovery_c r;
  r.setMaxCoastDist(40.0f);
  r.setMaxSearchTime(1500.0f);

  r.update(true, 0.5f, 0.0f, 0.0f);
  r.update(false, 0.5f, 10.0f, 100.0f);
  r.update(false, 0.5f, 55.0f, 500.0f);
  RecoveryAction stopped = r.update(false, 0.5f, 60.0f, 2100.0f);
  EXPECT(stopped.state == REC_STOPPED, "reached STOPPED");

  RecoveryAction a = r.update(true, 0.2f, 60.0f, 2200.0f);
  EXPECT(a.state == REC_FOLLOWING, "line seen from STOPPED -> FOLLOWING");
}

// reset() must return the machine to FOLLOWING and clear the remembered side.
static void test_reset() {
  LineRecovery_c r;
  r.update(true, -0.7f, 0.0f, 0.0f);
  r.update(false, -0.7f, 100.0f, 100.0f);   // drive it out of FOLLOWING
  EXPECT(r.getState() != REC_FOLLOWING, "moved out of FOLLOWING before reset");

  r.reset();
  EXPECT(r.getState() == REC_FOLLOWING, "reset returns to FOLLOWING");
  EXPECT(r.getLastSide() == 0, "reset clears the remembered side");
}

// A zero coast distance should escalate to searching immediately on loss.
static void test_zero_coast_dist_searches_immediately() {
  LineRecovery_c r;
  r.setMaxCoastDist(0.0f);
  r.setMaxSearchTime(1500.0f);

  r.update(true, 0.5f, 0.0f, 0.0f);
  RecoveryAction a = r.update(false, 0.5f, 0.0f, 100.0f);
  EXPECT(a.state == REC_SEARCHING, "zero coast distance -> immediate SEARCHING");
}

int main() {
  printf("Running test_recovery...\n");
  test_present_is_following();
  test_brief_loss_coasts();
  test_coast_limit_searches_right();
  test_search_sweeps_left_for_negative_error();
  test_search_limit_stops();
  test_reacquire_mid_search();
  test_reacquire_from_stopped();
  test_reset();
  test_zero_coast_dist_searches_immediately();
  return test_summary("test_recovery");
}
