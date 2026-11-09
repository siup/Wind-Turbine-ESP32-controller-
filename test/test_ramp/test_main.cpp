// Ramp: timing, rounding, sign change, zero ramp.
#include <unity.h>

#include "ramp.h"

void setUp() {}
void tearDown() {}

static void test_default_full_scale_500ms() {
  Ramp r;
  TEST_ASSERT_EQUAL_UINT32(500, r.fullScaleMs());
  r.setTarget(100);
  TEST_ASSERT_TRUE(r.update(10));
  TEST_ASSERT_EQUAL_INT32(2000, r.valueMilli());  // 2 % per 10 ms tick
  TEST_ASSERT_EQUAL_INT32(2, r.value());
  for (int i = 0; i < 48; i++) r.update(10);
  TEST_ASSERT_EQUAL_INT32(98, r.value());
  TEST_ASSERT_FALSE(r.settled());
  TEST_ASSERT_TRUE(r.update(10));
  TEST_ASSERT_TRUE(r.settled());
  TEST_ASSERT_EQUAL_INT32(100, r.value());
  TEST_ASSERT_FALSE(r.update(10));  // nothing left to do
}

static void test_partial_range_takes_proportional_time() {
  Ramp r;
  r.setFullScaleMs(1000);
  r.setTarget(30);
  uint32_t t = 0;
  while (!r.settled() && t < 5000) {
    r.update(10);
    t += 10;
  }
  TEST_ASSERT_EQUAL_UINT32(300, t);  // 30 % of 1000 ms
  TEST_ASSERT_EQUAL_INT32(30, r.value());
}

static void test_long_ramp_has_no_drift() {
  // 100000 milli-percent over 3000 ms is 33.33 per ms: the carry must make
  // the ramp land exactly on time, not early or late.
  Ramp r;
  r.setFullScaleMs(3000);
  r.setTarget(100);
  for (int i = 0; i < 2999; i++) r.update(1);
  TEST_ASSERT_FALSE(r.settled());
  TEST_ASSERT_EQUAL_INT32(99966, r.valueMilli());
  r.update(1);
  TEST_ASSERT_TRUE(r.settled());
  TEST_ASSERT_EQUAL_INT32(100000, r.valueMilli());
}

static void test_small_steps_accumulate() {
  // 10 s ramp, 1 ms ticks: 10 milli-percent each, never stuck at 0.
  Ramp r;
  r.setFullScaleMs(10000);
  r.setTarget(1);
  for (int i = 0; i < 99; i++) r.update(1);
  TEST_ASSERT_EQUAL_INT32(990, r.valueMilli());
  r.update(1);
  TEST_ASSERT_TRUE(r.settled());
}

static void test_rounding_half_away_from_zero() {
  Ramp r;
  r.setFullScaleMs(100000);  // 1 milli-percent per ms
  r.setTarget(100);
  r.update(1499);
  TEST_ASSERT_EQUAL_INT32(1, r.value());
  r.update(1);
  TEST_ASSERT_EQUAL_INT32(1500, r.valueMilli());
  TEST_ASSERT_EQUAL_INT32(2, r.value());

  Ramp n;
  n.setFullScaleMs(100000);
  n.setTarget(-100);
  n.update(1499);
  TEST_ASSERT_EQUAL_INT32(-1, n.value());
  n.update(1);
  TEST_ASSERT_EQUAL_INT32(-2, n.value());
  TEST_ASSERT_EQUAL_INT32(-100, n.target());
}

static void test_sign_change_passes_through_zero() {
  Ramp r;
  r.setFullScaleMs(1000);
  r.jumpTo(50);
  TEST_ASSERT_EQUAL_INT32(50, r.value());
  TEST_ASSERT_TRUE(r.settled());
  r.setTarget(-50);
  for (int i = 0; i < 50; i++) r.update(10);
  TEST_ASSERT_EQUAL_INT32(0, r.valueMilli());
  for (int i = 0; i < 49; i++) r.update(10);
  TEST_ASSERT_EQUAL_INT32(-49, r.value());
  r.update(10);
  TEST_ASSERT_EQUAL_INT32(-50, r.value());
  TEST_ASSERT_TRUE(r.settled());
}

static void test_zero_ramp_jumps() {
  Ramp r;
  r.setFullScaleMs(0);
  r.setTarget(80);
  TEST_ASSERT_TRUE(r.update(1));
  TEST_ASSERT_EQUAL_INT32(80, r.value());
  TEST_ASSERT_FALSE(r.update(1));
  r.setTarget(-30);
  TEST_ASSERT_TRUE(r.update(0));
  TEST_ASSERT_EQUAL_INT32(-30, r.value());
}

static void test_update_zero_dt_does_nothing() {
  Ramp r;
  r.setTarget(50);
  TEST_ASSERT_FALSE(r.update(0));
  TEST_ASSERT_EQUAL_INT32(0, r.valueMilli());
}

static void test_target_clamped() {
  Ramp r;
  r.setTarget(150);
  TEST_ASSERT_EQUAL_INT32(100, r.target());
  r.setTarget(-150);
  TEST_ASSERT_EQUAL_INT32(-100, r.target());
  r.setTargetMilli(123456);
  TEST_ASSERT_EQUAL_INT32(Ramp::kFull, r.targetMilli());
}

static void test_retarget_mid_ramp() {
  Ramp r;  // 500 ms
  r.setTarget(100);
  for (int i = 0; i < 25; i++) r.update(10);
  TEST_ASSERT_EQUAL_INT32(50, r.value());
  r.setTarget(20);  // ramp back down at the same rate
  for (int i = 0; i < 15; i++) r.update(10);
  TEST_ASSERT_EQUAL_INT32(20, r.value());
  TEST_ASSERT_TRUE(r.settled());
}

static void test_jump_to() {
  Ramp r;
  r.setTarget(100);
  r.update(10);
  r.jumpTo(0);
  TEST_ASSERT_EQUAL_INT32(0, r.valueMilli());
  TEST_ASSERT_EQUAL_INT32(0, r.targetMilli());
  TEST_ASSERT_TRUE(r.settled());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_default_full_scale_500ms);
  RUN_TEST(test_partial_range_takes_proportional_time);
  RUN_TEST(test_long_ramp_has_no_drift);
  RUN_TEST(test_small_steps_accumulate);
  RUN_TEST(test_rounding_half_away_from_zero);
  RUN_TEST(test_sign_change_passes_through_zero);
  RUN_TEST(test_zero_ramp_jumps);
  RUN_TEST(test_update_zero_dt_does_nothing);
  RUN_TEST(test_target_clamped);
  RUN_TEST(test_retarget_mid_ramp);
  RUN_TEST(test_jump_to);
  return UNITY_END();
}
