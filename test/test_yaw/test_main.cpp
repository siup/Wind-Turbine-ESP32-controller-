// Yaw logic: speed-weighted balance and twist limit, endstop references,
// debouncing, angle helpers, AS5600 unwrapping and the goto planner.
#include <unity.h>

#include <cmath>

#include "config.h"
#include "yaw_logic.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// YawBalance
// ---------------------------------------------------------------------------

static void test_balance_speed_weighting() {
  YawBalance b;
  b.reset();
  b.integrate(+1, 50, 1000);  // 1 s at 50 % = 500 ms at 100 %
  TEST_ASSERT_EQUAL_INT32(500, b.balance());
  b.integrate(-1, 100, 1000);
  TEST_ASSERT_EQUAL_INT32(-500, b.balance());
  // the same travel at any speed gives the same balance
  YawBalance slow, fast;
  slow.reset();
  fast.reset();
  for (int i = 0; i < 400; i++) slow.integrate(+1, 25, 10);
  for (int i = 0; i < 100; i++) fast.integrate(+1, 100, 10);
  TEST_ASSERT_EQUAL_INT32(fast.balance(), slow.balance());
  TEST_ASSERT_EQUAL_INT32(1000, fast.balance());
}

static void test_balance_keeps_sub_ms_remainders() {
  YawBalance b;
  b.reset();
  for (int i = 0; i < 1000; i++) b.integrate(+1, 1, 1);  // 0.01 ms each
  TEST_ASSERT_EQUAL_INT32(10, b.balance());
  YawBalance c;
  c.reset();
  for (int i = 0; i < 3; i++) c.integrate(+1, 33, 1);  // 0.99 ms
  TEST_ASSERT_EQUAL_INT32(1, c.balance());             // rounded
  c.reset();
  c.integrate(-1, 33, 1);  // -0.33
  TEST_ASSERT_EQUAL_INT32(0, c.balance());
  c.integrate(-1, 20, 1);  // -0.53
  TEST_ASSERT_EQUAL_INT32(-1, c.balance());
}

static void test_balance_ignores_no_motion() {
  YawBalance b;
  b.reset();
  b.integrate(0, 100, 1000);
  b.integrate(+1, 0, 1000);
  b.integrate(-1, 100, 0);
  TEST_ASSERT_EQUAL_INT32(0, b.balance());
  b.integrate(+1, 250, 100);  // clamped to 100 %
  TEST_ASSERT_EQUAL_INT32(100, b.balance());
}

static void test_balance_limits() {
  YawBalance b;
  b.reset();
  TEST_ASSERT_FALSE(b.blocked(+1, 0));  // 0 = off
  b.integrate(+1, 100, 999);
  TEST_ASSERT_FALSE(b.blocked(+1, 1000));
  b.integrate(+1, 100, 1);
  TEST_ASSERT_TRUE(b.blocked(+1, 1000));   // reached: no further right
  TEST_ASSERT_FALSE(b.blocked(-1, 1000));  // back is always allowed
  TEST_ASSERT_FALSE(b.blocked(0, 1000));
  TEST_ASSERT_FALSE(b.blocked(+1, 0));
  b.integrate(-1, 100, 3000);
  TEST_ASSERT_EQUAL_INT32(-2000, b.balance());
  TEST_ASSERT_TRUE(b.blocked(-1, 1000));
  TEST_ASSERT_FALSE(b.blocked(+1, 1000));
  TEST_ASSERT_TRUE(b.blocked(-1, 2000));
  TEST_ASSERT_FALSE(b.blocked(-1, 2001));
}

static void test_balance_restore_and_estimate() {
  YawBalance b;
  b.restore(-1520);
  TEST_ASSERT_EQUAL_INT32(-1520, b.balance());
  float deg = 0;
  TEST_ASSERT_FALSE(b.estimateDeg(0, deg));  // unknown ms-per-360
  b.restore(500);
  TEST_ASSERT_TRUE(b.estimateDeg(2000, deg));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, deg);
  b.restore(-5000);
  TEST_ASSERT_TRUE(b.estimateDeg(2000, deg));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -900.0f, deg);  // continuous, beyond -360
  b.reset();
  b.integrate(+1, 50, 1);  // 0.5 ms
  TEST_ASSERT_TRUE(b.estimateDeg(360, deg));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, deg);  // sub-ms precision kept
}

static void test_endstop_reference_snapping() {
  YawBalance b;
  b.reset();
  TEST_ASSERT_FALSE(b.hasReference(-1));
  TEST_ASSERT_FALSE(b.hasReference(+1));
  b.integrate(+1, 100, 4000);
  b.onEndstop(+1);  // first hit: remember, do not change
  TEST_ASSERT_TRUE(b.hasReference(+1));
  TEST_ASSERT_FALSE(b.hasReference(-1));
  TEST_ASSERT_EQUAL_INT32(4000, b.balance());

  // go left and come back with drift: the next hit snaps back
  b.integrate(-1, 100, 3000);
  b.integrate(+1, 100, 3150);
  TEST_ASSERT_EQUAL_INT32(4150, b.balance());
  b.onEndstop(+1);
  TEST_ASSERT_EQUAL_INT32(4000, b.balance());

  // left side works the same way, independently
  b.integrate(-1, 100, 8000);
  b.onEndstop(-1);
  TEST_ASSERT_EQUAL_INT32(-4000, b.balance());
  b.integrate(+1, 100, 100);
  b.integrate(-1, 100, 50);
  b.onEndstop(-1);
  TEST_ASSERT_EQUAL_INT32(-4000, b.balance());
  b.onEndstop(0);  // not a side
  TEST_ASSERT_EQUAL_INT32(-4000, b.balance());
  TEST_ASSERT_FALSE(b.hasReference(0));

  // reset() and restore() forget the references
  b.reset();
  TEST_ASSERT_FALSE(b.hasReference(+1));
  TEST_ASSERT_FALSE(b.hasReference(-1));
  b.integrate(+1, 100, 1000);
  b.onEndstop(+1);
  b.restore(200);
  TEST_ASSERT_FALSE(b.hasReference(+1));
  b.onEndstop(+1);
  TEST_ASSERT_EQUAL_INT32(200, b.balance());
}

// ---------------------------------------------------------------------------
// EndstopDebouncer
// ---------------------------------------------------------------------------

static void test_debounce_press_and_release() {
  EndstopDebouncer d;
  d.reset(false, 0);
  TEST_ASSERT_FALSE(d.active());
  for (uint32_t t = 0; t < ENDSTOP_DEBOUNCE_MS; t++) {
    TEST_ASSERT_FALSE(d.update(true, t, ENDSTOP_DEBOUNCE_MS));
    TEST_ASSERT_FALSE(d.rose());
  }
  TEST_ASSERT_TRUE(d.update(true, ENDSTOP_DEBOUNCE_MS, ENDSTOP_DEBOUNCE_MS));
  TEST_ASSERT_TRUE(d.rose());
  TEST_ASSERT_TRUE(d.update(true, ENDSTOP_DEBOUNCE_MS + 1, ENDSTOP_DEBOUNCE_MS));
  TEST_ASSERT_FALSE(d.rose());  // only once
  // release is debounced too, and does not set rose()
  TEST_ASSERT_TRUE(d.update(false, 100, ENDSTOP_DEBOUNCE_MS));
  TEST_ASSERT_TRUE(d.update(false, 119, ENDSTOP_DEBOUNCE_MS));
  TEST_ASSERT_FALSE(d.update(false, 120, ENDSTOP_DEBOUNCE_MS));
  TEST_ASSERT_FALSE(d.rose());
}

static void test_debounce_bounce_restarts_timer() {
  EndstopDebouncer d;
  d.reset(false, 1000);
  d.update(true, 1000, 20);
  d.update(false, 1005, 20);  // contact bounce
  d.update(true, 1010, 20);
  TEST_ASSERT_FALSE(d.update(true, 1025, 20));
  TEST_ASSERT_TRUE(d.update(true, 1030, 20));
  TEST_ASSERT_TRUE(d.rose());
  // a short glitch while pressed does not release
  d.update(false, 1040, 20);
  TEST_ASSERT_TRUE(d.update(true, 1045, 20));
  TEST_ASSERT_TRUE(d.update(true, 1100, 20));
  TEST_ASSERT_FALSE(d.rose());
}

static void test_debounce_initial_state_and_wrap() {
  EndstopDebouncer d;
  d.reset(true, 0);  // already pressed at boot
  TEST_ASSERT_TRUE(d.active());
  TEST_ASSERT_TRUE(d.update(true, 5, 20));
  TEST_ASSERT_FALSE(d.rose());  // no edge at boot

  EndstopDebouncer w;  // millis() wrap
  w.reset(false, 0xFFFFFFF8u);
  w.update(true, 0xFFFFFFFCu, 20);
  TEST_ASSERT_FALSE(w.update(true, 0x0000000Fu, 20));  // 19 ms
  TEST_ASSERT_TRUE(w.update(true, 0x00000010u, 20));   // 20 ms
  EndstopDebouncer z;
  z.reset(false, 0);
  TEST_ASSERT_TRUE(z.update(true, 0, 0));  // no debounce
  TEST_ASSERT_TRUE(z.rose());
}

// ---------------------------------------------------------------------------
// Angle helpers
// ---------------------------------------------------------------------------

static void test_median3() {
  const float v[3] = {1.0f, 2.0f, 3.0f};
  const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (auto& p : perms) TEST_ASSERT_EQUAL_FLOAT(2.0f, median3(v[p[0]], v[p[1]], v[p[2]]));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, median3(5.0f, 5.0f, 1.0f));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, median3(1.0f, 5.0f, 5.0f));
  TEST_ASSERT_EQUAL_FLOAT(7.0f, median3(7.0f, 7.0f, 7.0f));
  TEST_ASSERT_EQUAL_FLOAT(100.0f, median3(100.0f, 359.0f, 99.0f));  // a spike is rejected
}

static void test_wrap360() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(0.0f));
  TEST_ASSERT_EQUAL_FLOAT(350.0f, wrap360(-10.0f));
  TEST_ASSERT_EQUAL_FLOAT(10.0f, wrap360(370.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(360.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(-360.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(720.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 359.5f, wrap360(-0.5f));
  float tiny = wrap360(-1e-6f);
  TEST_ASSERT_TRUE(tiny >= 0.0f && tiny < 360.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(NAN));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap360(INFINITY));
}

static void test_shortest_delta() {
  TEST_ASSERT_EQUAL_FLOAT(20.0f, shortestDelta(350.0f, 10.0f));
  TEST_ASSERT_EQUAL_FLOAT(-20.0f, shortestDelta(10.0f, 350.0f));
  TEST_ASSERT_EQUAL_FLOAT(180.0f, shortestDelta(0.0f, 180.0f));  // (-180, 180]
  TEST_ASSERT_EQUAL_FLOAT(180.0f, shortestDelta(180.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(180.0f, shortestDelta(90.0f, 270.0f));
  TEST_ASSERT_EQUAL_FLOAT(-179.0f, shortestDelta(0.0f, 181.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, shortestDelta(42.0f, 42.0f));
  TEST_ASSERT_EQUAL_FLOAT(-90.0f, shortestDelta(720.0f, 270.0f));  // inputs wrapped
}

// ---------------------------------------------------------------------------
// AngleUnwrapper
// ---------------------------------------------------------------------------

static void test_unwrap_across_zero() {
  AngleUnwrapper u;
  u.reset(0.0f);
  TEST_ASSERT_FALSE(u.initialized());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, u.update(10.0f));
  TEST_ASSERT_TRUE(u.initialized());
  TEST_ASSERT_EQUAL_FLOAT(-10.0f, u.update(350.0f));  // left across 0
  TEST_ASSERT_EQUAL_FLOAT(-20.0f, u.update(340.0f));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, u.update(5.0f));      // right across 0
  // a full right turn and a bit
  const float seq[] = {90.0f, 180.0f, 270.0f, 359.0f, 10.0f};
  for (float a : seq) u.update(a);
  TEST_ASSERT_EQUAL_FLOAT(370.0f, u.continuous());
  // two full left turns
  for (int turn = 0; turn < 2; turn++) {
    const float back[] = {270.0f, 180.0f, 90.0f, 0.0f};
    for (float a : back) u.update(a);
  }
  TEST_ASSERT_EQUAL_FLOAT(-360.0f, u.continuous());
}

static void test_unwrap_starts_near_hint() {
  AngleUnwrapper u;
  u.reset(725.0f);
  TEST_ASSERT_EQUAL_FLOAT(725.0f, u.continuous());  // before the first reading
  TEST_ASSERT_EQUAL_FLOAT(730.0f, u.update(10.0f));
  u.reset(-5.0f);
  TEST_ASSERT_EQUAL_FLOAT(-10.0f, u.update(350.0f));
  u.reset(-700.0f);
  TEST_ASSERT_EQUAL_FLOAT(-710.0f, u.update(10.0f));  // -720 + 10
  u.reset(179.0f);
  TEST_ASSERT_EQUAL_FLOAT(190.0f, u.update(190.0f));  // nearest to 179, not -170
}

static void test_unwrap_no_drift() {
  AngleUnwrapper u;
  u.reset(0.0f);
  u.update(0.0f);
  // 50 turns right in 7.3 degree steps, then back to an exact reading
  float abs = 0.0f;
  for (int i = 0; i < 2466; i++) {
    abs = wrap360(abs + 7.3f);
    u.update(abs);
  }
  u.update(0.0f);
  float turns = u.continuous() / 360.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, std::round(turns), turns);
  TEST_ASSERT_EQUAL_FLOAT(50.0f * 360.0f, u.continuous());
}

// ---------------------------------------------------------------------------
// planGoto
// ---------------------------------------------------------------------------

static GotoParams params() {
  GotoParams p;
  p.windowDeg = YAW_GOTO_WINDOW_DEG;
  p.slowZoneDeg = YAW_GOTO_SLOW_DEG;
  p.slowPct = YAW_GOTO_SLOW_PCT;
  return p;
}

static void test_goto_shortest_way() {
  GotoStep s = planGoto(10.0f, 30.0f, 60, 0.0f, params());
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
  TEST_ASSERT_EQUAL_UINT8(60, s.speedPct);
  TEST_ASSERT_FALSE(s.done);
  TEST_ASSERT_FALSE(s.impossible);
  s = planGoto(30.0f, 10.0f, 60, 0.0f, params());
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  s = planGoto(350.0f, 10.0f, 60, 0.0f, params());  // across 0
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
  s = planGoto(10.0f, 350.0f, 60, 0.0f, params());
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  s = planGoto(-350.0f, 300.0f, 60, 0.0f, params());  // continuous input: abs 10 -> 300 is -70
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  s = planGoto(725.0f, 0.0f, 60, 0.0f, params());     // abs 5 -> 0
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  TEST_ASSERT_EQUAL_UINT8(YAW_GOTO_SLOW_PCT, s.speedPct);
}

static void test_goto_window_and_slow_zone() {
  GotoStep s = planGoto(100.0f, 101.5f, 60, 0.0f, params());
  TEST_ASSERT_TRUE(s.done);
  TEST_ASSERT_EQUAL_INT8(0, s.dir);
  TEST_ASSERT_EQUAL_UINT8(0, s.speedPct);
  s = planGoto(100.0f, 98.0f, 60, 0.0f, params());  // exactly on the window edge
  TEST_ASSERT_TRUE(s.done);
  s = planGoto(359.0f, 0.5f, 60, 0.0f, params());  // window across 0
  TEST_ASSERT_TRUE(s.done);
  s = planGoto(100.0f, 108.0f, 60, 0.0f, params());
  TEST_ASSERT_FALSE(s.done);
  TEST_ASSERT_EQUAL_UINT8(30, s.speedPct);
  s = planGoto(100.0f, 110.0f, 60, 0.0f, params());  // slow zone edge
  TEST_ASSERT_EQUAL_UINT8(30, s.speedPct);
  s = planGoto(100.0f, 111.0f, 60, 0.0f, params());
  TEST_ASSERT_EQUAL_UINT8(60, s.speedPct);
  // never below slowPct, never above 100
  s = planGoto(100.0f, 200.0f, 20, 0.0f, params());
  TEST_ASSERT_EQUAL_UINT8(30, s.speedPct);
  s = planGoto(100.0f, 200.0f, 0, 0.0f, params());
  TEST_ASSERT_EQUAL_UINT8(30, s.speedPct);
  s = planGoto(100.0f, 200.0f, 150, 0.0f, params());
  TEST_ASSERT_EQUAL_UINT8(100, s.speedPct);
}

static void test_goto_long_way_under_twist_limit() {
  // at +300, target 90: the short way (+150) would end at 450 > 360
  GotoStep s = planGoto(300.0f, 90.0f, 60, 360.0f, params());
  TEST_ASSERT_FALSE(s.impossible);
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);  // long way back to +90
  TEST_ASSERT_EQUAL_UINT8(60, s.speedPct);
  // the decision stays stable while travelling
  for (float cont = 290.0f; cont > 100.0f; cont -= 10.0f) {
    s = planGoto(cont, 90.0f, 60, 360.0f, params());
    TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  }
  s = planGoto(91.0f, 90.0f, 60, 360.0f, params());
  TEST_ASSERT_TRUE(s.done);
  // mirrored: at -300 (abs 60) going to 270: the short way -150 would end at
  // -450, so the long way +210 to -90
  s = planGoto(-300.0f, 270.0f, 60, 360.0f, params());
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
  s = planGoto(-300.0f, 270.0f, 60, 0.0f, params());  // no limit: short way
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  // at -350 (abs 10) to 350: short way -20 ends at -370, so +340 to -10
  s = planGoto(-350.0f, 350.0f, 60, 360.0f, params());
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
  // a twist limit that is not in the way changes nothing
  s = planGoto(10.0f, 30.0f, 60, 360.0f, params());
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
  // ending exactly on the limit is allowed
  s = planGoto(340.0f, 0.0f, 60, 360.0f, params());
  TEST_ASSERT_EQUAL_INT8(1, s.dir);
}

static void test_goto_impossible() {
  // limit 100: from 0 to 180 is 180 either way
  GotoStep s = planGoto(0.0f, 180.0f, 60, 100.0f, params());
  TEST_ASSERT_TRUE(s.impossible);
  TEST_ASSERT_EQUAL_INT8(0, s.dir);
  TEST_ASSERT_EQUAL_UINT8(0, s.speedPct);
  TEST_ASSERT_FALSE(s.done);
  // already inside the window is done, whatever the limit says
  s = planGoto(500.0f, 140.0f, 60, 100.0f, params());
  TEST_ASSERT_TRUE(s.done);
  TEST_ASSERT_FALSE(s.impossible);
  // beyond the limit already: the way back in is allowed
  s = planGoto(400.0f, 0.0f, 60, 360.0f, params());
  TEST_ASSERT_FALSE(s.impossible);
  TEST_ASSERT_EQUAL_INT8(-1, s.dir);
  // bad input
  s = planGoto(NAN, 0.0f, 60, 0.0f, params());
  TEST_ASSERT_TRUE(s.impossible);
  TEST_ASSERT_EQUAL_INT8(0, s.dir);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_balance_speed_weighting);
  RUN_TEST(test_balance_keeps_sub_ms_remainders);
  RUN_TEST(test_balance_ignores_no_motion);
  RUN_TEST(test_balance_limits);
  RUN_TEST(test_balance_restore_and_estimate);
  RUN_TEST(test_endstop_reference_snapping);
  RUN_TEST(test_debounce_press_and_release);
  RUN_TEST(test_debounce_bounce_restarts_timer);
  RUN_TEST(test_debounce_initial_state_and_wrap);
  RUN_TEST(test_median3);
  RUN_TEST(test_wrap360);
  RUN_TEST(test_shortest_delta);
  RUN_TEST(test_unwrap_across_zero);
  RUN_TEST(test_unwrap_starts_near_hint);
  RUN_TEST(test_unwrap_no_drift);
  RUN_TEST(test_goto_shortest_way);
  RUN_TEST(test_goto_window_and_slow_zone);
  RUN_TEST(test_goto_long_way_under_twist_limit);
  RUN_TEST(test_goto_impossible);
  return UNITY_END();
}
