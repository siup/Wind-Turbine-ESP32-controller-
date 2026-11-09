// Battery math: divider conversion, percentage, moving average, and the
// low / critical state machine (confirm count, hysteresis, absent battery).
#include <unity.h>

#include "battery_math.h"
#include "config.h"

void setUp() {}
void tearDown() {}

static BatteryMonitor monitor9V() {
  BatteryMonitor m;
  BatteryMonitor::Config c;
  c.lowMv = BAT9V_LOW_MV;     // 7000
  c.critMv = BAT9V_CRIT_MV;   // 6200
  c.critHystMv = BAT_CRIT_HYST_MV;
  c.lowHystMv = BAT_LOW_HYST_MV;
  c.critConfirm = BAT_CRIT_CONFIRM;
  c.presentMinMv = BAT_PRESENT_MIN_MV;
  m.configure(c);
  m.reset();
  return m;
}

static void test_mv_from_adc() {
  // 100k / 33k: x 133/33 = 4.0303
  TEST_ASSERT_EQUAL_UINT32(7658, batteryMvFromAdc(1900, 100000.0f, 33000.0f, 1.0f));
  TEST_ASSERT_EQUAL_UINT32(8423, batteryMvFromAdc(1900, 100000.0f, 33000.0f, 1.1f));
  TEST_ASSERT_EQUAL_UINT32(0, batteryMvFromAdc(0, BAT_DIV_R_TOP, BAT_DIV_R_BOTTOM, BAT_CAL));
  TEST_ASSERT_EQUAL_UINT32(3100, batteryMvFromAdc(3100, 0.0f, 33000.0f, 1.0f));  // no divider
  TEST_ASSERT_EQUAL_UINT32(0, batteryMvFromAdc(1900, 100000.0f, 0.0f, 1.0f));    // nonsense
  TEST_ASSERT_EQUAL_UINT32(0, batteryMvFromAdc(1900, 100000.0f, 33000.0f, 0.0f));
  // full ADC range of the S3 / ESP32 (about 3.1 V) stays well within uint32
  TEST_ASSERT_EQUAL_UINT32(12494, batteryMvFromAdc(3100, BAT_DIV_R_TOP, BAT_DIV_R_BOTTOM, 1.0f));
}

static void test_percent() {
  TEST_ASSERT_EQUAL_UINT8(0, batteryPercent(5000, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(0, batteryPercent(6200, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(50, batteryPercent(7800, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(100, batteryPercent(9400, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(100, batteryPercent(12000, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(1, batteryPercent(6232, 6200, 9400));   // 1.0 %
  TEST_ASSERT_EQUAL_UINT8(1, batteryPercent(6216, 6200, 9400));   // 0.5 % rounds up
  TEST_ASSERT_EQUAL_UINT8(0, batteryPercent(6215, 6200, 9400));
  TEST_ASSERT_EQUAL_UINT8(99, batteryPercent(9383, 6200, 9400));  // 99.47 %
  // misconfigured range never divides by zero
  TEST_ASSERT_EQUAL_UINT8(0, batteryPercent(7000, 8000, 8000));
  TEST_ASSERT_EQUAL_UINT8(100, batteryPercent(9000, 8000, 7000));
}

static void test_moving_average() {
  MovingAverage<4> a;
  TEST_ASSERT_EQUAL_UINT32(0, a.mean());
  TEST_ASSERT_EQUAL_UINT8(0, a.count());
  a.push(10);
  TEST_ASSERT_EQUAL_UINT32(10, a.mean());
  a.push(11);
  TEST_ASSERT_EQUAL_UINT32(11, a.mean());  // 10.5 rounds up
  a.push(20);
  a.push(30);
  TEST_ASSERT_TRUE(a.full());
  TEST_ASSERT_EQUAL_UINT32(18, a.mean());  // 71 / 4 = 17.75
  a.push(100);                             // drops 10
  TEST_ASSERT_EQUAL_UINT32(40, a.mean());  // 161 / 4 = 40.25
  TEST_ASSERT_EQUAL_UINT8(4, a.count());
  a.reset();
  TEST_ASSERT_EQUAL_UINT8(0, a.count());
  a.push(7);
  TEST_ASSERT_EQUAL_UINT32(7, a.mean());

  MovingAverage<BAT_AVG_SAMPLES> big;
  for (int i = 0; i < 1000; i++) big.push(0xFFFFFFFFu);  // no overflow
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, big.mean());
}

static void test_normal_voltage() {
  BatteryMonitor m = monitor9V();
  TEST_ASSERT_FALSE(m.update(8000));
  TEST_ASSERT_FALSE(m.low());
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.present());
}

static void test_crit_needs_confirmation() {
  BatteryMonitor m = monitor9V();
  TEST_ASSERT_FALSE(m.update(6100));  // first reading below crit
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.low());
  TEST_ASSERT_TRUE(m.update(6100));  // second: latches, reported once
  TEST_ASSERT_TRUE(m.crit());
  TEST_ASSERT_FALSE(m.update(6000));  // no change
  TEST_ASSERT_TRUE(m.crit());
}

static void test_crit_run_must_be_consecutive() {
  BatteryMonitor m = monitor9V();
  m.update(6100);
  m.update(6250);  // a dip that recovered: count restarts
  TEST_ASSERT_FALSE(m.update(6100));
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.update(6100));
  TEST_ASSERT_TRUE(m.crit());
}

static void test_crit_exactly_at_threshold_is_not_crit() {
  BatteryMonitor m = monitor9V();
  for (int i = 0; i < 5; i++) m.update(6200);
  TEST_ASSERT_FALSE(m.crit());
}

static void test_crit_hysteresis() {
  BatteryMonitor m = monitor9V();
  m.update(6000);
  m.update(6000);
  TEST_ASSERT_TRUE(m.crit());
  TEST_ASSERT_FALSE(m.update(6300));  // above crit, below crit + 0.3 V
  TEST_ASSERT_FALSE(m.update(6499));
  TEST_ASSERT_TRUE(m.crit());
  TEST_ASSERT_TRUE(m.update(6500));  // crit + 300 mV releases
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.low());  // still below low
}

static void test_low_hysteresis() {
  BatteryMonitor m = monitor9V();
  m.update(6990);
  TEST_ASSERT_TRUE(m.low());
  m.update(7050);
  TEST_ASSERT_TRUE(m.low());  // inside the 100 mV band
  m.update(7099);
  TEST_ASSERT_TRUE(m.low());
  m.update(7100);
  TEST_ASSERT_FALSE(m.low());
  m.update(7000);  // at the threshold: not low
  TEST_ASSERT_FALSE(m.low());
  m.update(6999);
  TEST_ASSERT_TRUE(m.low());
}

static void test_low_while_crit() {
  BatteryMonitor::Config c;
  c.lowMv = 7000;
  c.critMv = 6200;
  c.critHystMv = 2000;  // releases at 8200, above low + hyst
  c.critConfirm = 1;
  BatteryMonitor m;
  m.configure(c);
  m.reset();
  m.update(6000);
  TEST_ASSERT_TRUE(m.crit());
  m.update(7500);  // above low + hyst, but still crit
  TEST_ASSERT_TRUE(m.crit());
  TEST_ASSERT_TRUE(m.low());
  m.update(8200);
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_FALSE(m.low());
}

static void test_absent_battery() {
  BatteryMonitor m = monitor9V();
  for (int i = 0; i < 10; i++) TEST_ASSERT_FALSE(m.update(100));  // USB power
  TEST_ASSERT_FALSE(m.present());
  TEST_ASSERT_FALSE(m.low());
  TEST_ASSERT_FALSE(m.crit());  // no new latch while absent
  m.update(BAT_PRESENT_MIN_MV - 1);
  TEST_ASSERT_FALSE(m.present());
  m.update(BAT_PRESENT_MIN_MV);
  TEST_ASSERT_TRUE(m.present());  // 2.5 V counts as present (and is crit-low)
  TEST_ASSERT_TRUE(m.low());
}

static void test_absent_never_clears_crit() {
  BatteryMonitor m = monitor9V();
  m.update(6000);
  m.update(6000);
  TEST_ASSERT_TRUE(m.crit());
  for (int i = 0; i < 20; i++) {
    TEST_ASSERT_FALSE(m.update(0));  // divider wire fell off
    TEST_ASSERT_TRUE(m.crit());
    TEST_ASSERT_FALSE(m.present());
  }
  TEST_ASSERT_TRUE(m.low());  // low follows crit
  m.update(6400);  // back, still within the hysteresis band
  TEST_ASSERT_TRUE(m.crit());
  TEST_ASSERT_TRUE(m.update(9000));  // fresh battery
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_FALSE(m.low());
}

static void test_absent_breaks_confirm_run() {
  BatteryMonitor m = monitor9V();
  m.update(6100);
  m.update(0);
  TEST_ASSERT_FALSE(m.update(6100));
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.update(6100));
  TEST_ASSERT_TRUE(m.crit());
}

static void test_confirm_zero_means_one() {
  BatteryMonitor::Config c;
  c.critConfirm = 0;
  BatteryMonitor m;
  m.configure(c);
  m.reset();
  TEST_ASSERT_TRUE(m.update(c.critMv - 1));
  TEST_ASSERT_TRUE(m.crit());
  c.critConfirm = 3;
  m.configure(c);
  m.reset();
  TEST_ASSERT_FALSE(m.crit());
  m.update(5000);
  m.update(5000);
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_TRUE(m.update(5000));
}

static void test_reset() {
  BatteryMonitor m = monitor9V();
  m.update(6000);
  m.update(6000);
  m.update(0);
  m.reset();
  TEST_ASSERT_FALSE(m.crit());
  TEST_ASSERT_FALSE(m.low());
  TEST_ASSERT_TRUE(m.present());
  TEST_ASSERT_EQUAL_UINT16(BAT9V_CRIT_MV, m.config().critMv);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_mv_from_adc);
  RUN_TEST(test_percent);
  RUN_TEST(test_moving_average);
  RUN_TEST(test_normal_voltage);
  RUN_TEST(test_crit_needs_confirmation);
  RUN_TEST(test_crit_run_must_be_consecutive);
  RUN_TEST(test_crit_exactly_at_threshold_is_not_crit);
  RUN_TEST(test_crit_hysteresis);
  RUN_TEST(test_low_hysteresis);
  RUN_TEST(test_low_while_crit);
  RUN_TEST(test_absent_battery);
  RUN_TEST(test_absent_never_clears_crit);
  RUN_TEST(test_absent_breaks_confirm_run);
  RUN_TEST(test_confirm_zero_means_one);
  RUN_TEST(test_reset);
  return UNITY_END();
}
