// Failsafe state machine: owner timeout, disconnects, battery / OTA,
// runtime latch, priorities, allowMotion.
#include <unity.h>

#include "config.h"
#include "failsafe_logic.h"

using fs::ClientRef;
using fs::FailsafeLogic;
using fs::Inputs;
using fs::Result;
using proto::Err;
using proto::FsReason;
using proto::Source;

void setUp() {}
void tearDown() {}

static const ClientRef A{Source::Ws, 1};
static const ClientRef B{Source::Ws, 2};
static const ClientRef BLE{Source::Ble, 1};  // same id as A, other transport

static Inputs at(uint32_t now, uint16_t clients = 1) {
  Inputs in;
  in.nowMs = now;
  in.clients = clients;
  return in;
}

static FailsafeLogic make() {
  FailsafeLogic f;
  f.setTimeout(FS_TIMEOUT_MS);
  return f;
}

static void assertQuiet(const Result& r) {
  TEST_ASSERT_FALSE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::None, (int)r.trippedReason);
  TEST_ASSERT_FALSE(r.zeroSetpoints);
  TEST_ASSERT_FALSE(r.hardStop);
}

static void test_client_ref() {
  TEST_ASSERT_FALSE(ClientRef{}.valid());
  TEST_ASSERT_TRUE(A.valid());
  TEST_ASSERT_TRUE(A == ClientRef(A));
  TEST_ASSERT_TRUE(A != B);
  TEST_ASSERT_TRUE(A != BLE);
}

static void test_idle_never_times_out() {
  FailsafeLogic f = make();
  assertQuiet(f.update(at(0)));
  assertQuiet(f.update(at(100000)));  // no owner, nothing supervised
  TEST_ASSERT_FALSE(f.active());
  TEST_ASSERT_EQUAL_INT((int)FsReason::None, (int)f.reason());
  TEST_ASSERT_EQUAL_INT((int)Err::None, (int)f.allowMotion());
}

static void test_owner_timeout() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 1000);
  TEST_ASSERT_TRUE(f.hasOwner());
  TEST_ASSERT_TRUE(f.owner() == A);
  assertQuiet(f.update(at(1500)));  // exactly the timeout: still fine
  Result r = f.update(at(1501));
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Timeout, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_FALSE(r.hardStop);
  TEST_ASSERT_FALSE(f.hasOwner());  // released
  TEST_ASSERT_TRUE(f.active());
  TEST_ASSERT_EQUAL_INT((int)FsReason::Timeout, (int)f.reason());
  TEST_ASSERT_TRUE(f.has(FsReason::Timeout));
  TEST_ASSERT_EQUAL_UINT8(1u << (int)FsReason::Timeout, f.mask());
  // the trip is reported once; nothing is restarted later
  assertQuiet(f.update(at(1550)));
  assertQuiet(f.update(at(5000)));
  TEST_ASSERT_TRUE(f.active());
  // timeout does not block motion: the next frame clears it
  TEST_ASSERT_EQUAL_INT((int)Err::None, (int)f.allowMotion());
  f.onFrame(A, 5001);
  TEST_ASSERT_FALSE(f.active());
}

static void test_owner_frames_keep_alive() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  for (uint32_t t = 300; t <= 3000; t += 300) {
    f.onFrame(A, t);  // ping every 300 ms
    assertQuiet(f.update(at(t + 50)));
  }
  TEST_ASSERT_FALSE(f.active());
  TEST_ASSERT_TRUE(f.update(at(3000 + 501)).tripped);
}

static void test_other_clients_do_not_keep_owner_alive() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  for (uint32_t t = 100; t <= 500; t += 100) {
    f.onFrame(B, t);    // another phone pings happily
    f.onFrame(BLE, t);  // same id, other transport: not the owner either
    assertQuiet(f.update(at(t, 2)));
  }
  Result r = f.update(at(501, 2));
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Timeout, (int)r.trippedReason);
  // ...but any client's next frame clears the trip
  f.onFrame(B, 600);
  TEST_ASSERT_FALSE(f.active());
}

static void test_new_owner_takes_over() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  f.onMotionCommand(B, 400);
  TEST_ASSERT_TRUE(f.owner() == B);
  assertQuiet(f.update(at(800, 2)));  // A silent 800 ms: not the owner any more
  f.onFrame(A, 850);
  TEST_ASSERT_TRUE(f.update(at(901, 2)).tripped);  // B silent 501 ms
}

static void test_custom_timeout() {
  FailsafeLogic f;
  f.setTimeout(800);
  TEST_ASSERT_EQUAL_UINT32(800, f.timeout());
  f.onMotionCommand(A, 0);
  assertQuiet(f.update(at(800)));
  TEST_ASSERT_TRUE(f.update(at(801)).tripped);
}

static void test_timestamps_out_of_order_and_wrap() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 1000);
  assertQuiet(f.update(at(990)));  // loop time captured before the frame arrived
  TEST_ASSERT_FALSE(f.active());

  FailsafeLogic w = make();
  w.onMotionCommand(A, 0xFFFFFF00u);
  assertQuiet(w.update(at(0x00000010u)));  // 272 ms across the millis() wrap
  Result r = w.update(at(0x00000101u));    // 513 ms
  TEST_ASSERT_TRUE(r.tripped);
}

static void test_release_owner_stops_supervision() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  f.releaseOwner();  // motors idle
  TEST_ASSERT_FALSE(f.hasOwner());
  assertQuiet(f.update(at(10000)));
}

static void test_last_client_disconnect_edge() {
  FailsafeLogic f = make();
  assertQuiet(f.update(at(0, 0)));  // boot with nobody: no edge
  assertQuiet(f.update(at(100, 2)));
  assertQuiet(f.update(at(150, 1)));  // one of two left: no trip without an owner
  Result r = f.update(at(200, 0));
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)f.reason());
  assertQuiet(f.update(at(250, 0)));  // an edge, not a state
  assertQuiet(f.update(at(300, 1)));  // reconnecting alone does not clear it
  TEST_ASSERT_TRUE(f.active());
  f.onFrame(A, 310);  // the first valid frame does
  TEST_ASSERT_FALSE(f.active());
}

static void test_disconnect_edge_with_owner() {
  FailsafeLogic f = make();
  f.update(at(0, 1));
  f.onMotionCommand(A, 10);
  Result r = f.update(at(20, 0));
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_FALSE(f.hasOwner());
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)f.reason());
  assertQuiet(f.update(at(2000, 0)));  // owner released: no timeout on top
  TEST_ASSERT_FALSE(f.has(FsReason::Timeout));
}

static void test_owner_disconnect() {
  FailsafeLogic f = make();
  f.update(at(0, 2));
  f.onMotionCommand(A, 10);
  f.onClientDisconnected(B, 20);  // not the owner: nothing
  assertQuiet(f.update(at(30, 1)));
  f.onClientDisconnected(A, 40);
  TEST_ASSERT_FALSE(f.active());  // applied on the next update
  Result r = f.update(at(50, 1));  // B (or someone) is still connected
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_FALSE(f.hasOwner());
  assertQuiet(f.update(at(60, 1)));  // reported once
  f.onFrame(B, 70);
  TEST_ASSERT_FALSE(f.active());
}

static void test_owner_disconnect_after_release_is_harmless() {
  FailsafeLogic f = make();
  f.update(at(0, 2));
  f.onMotionCommand(A, 10);
  f.onClientDisconnected(A, 20);
  f.releaseOwner();  // everything stopped in the same loop pass
  assertQuiet(f.update(at(30, 1)));
  // and a disconnect of a former owner later is ignored too
  f.onMotionCommand(A, 40);
  f.releaseOwner();
  f.onClientDisconnected(A, 50);
  assertQuiet(f.update(at(60, 1)));
}

static void test_new_motion_supersedes_pending_owner_loss() {
  FailsafeLogic f = make();
  f.update(at(0, 2));
  f.onMotionCommand(A, 10);
  f.onClientDisconnected(A, 20);
  f.onMotionCommand(B, 25);  // B drives now
  assertQuiet(f.update(at(30, 1)));
  TEST_ASSERT_TRUE(f.owner() == B);
}

static void test_battery() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  Inputs in = at(10);
  in.batteryCrit = true;
  Result r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Battery, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.hardStop);
  TEST_ASSERT_FALSE(f.hasOwner());
  TEST_ASSERT_EQUAL_INT((int)Err::LowBattery, (int)f.allowMotion());
  // still crit: no new trip, the stop request is repeated
  in.nowMs = 60;
  r = f.update(in);
  TEST_ASSERT_FALSE(r.tripped);
  TEST_ASSERT_TRUE(r.hardStop);
  // frames do not clear it
  f.onFrame(A, 70);
  TEST_ASSERT_TRUE(f.has(FsReason::Battery));
  TEST_ASSERT_EQUAL_INT((int)Err::LowBattery, (int)f.allowMotion());
  // the battery module released its latch (hysteresis lives there)
  in.batteryCrit = false;
  in.nowMs = 80;
  assertQuiet(f.update(in));
  TEST_ASSERT_FALSE(f.active());
  TEST_ASSERT_EQUAL_INT((int)Err::None, (int)f.allowMotion());
}

static void test_ota() {
  FailsafeLogic f = make();
  Inputs in = at(0);
  in.otaActive = true;
  Result r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Ota, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.hardStop);
  TEST_ASSERT_EQUAL_INT((int)Err::BusyOta, (int)f.allowMotion());
  in.batteryCrit = true;  // OTA wins over battery
  in.nowMs = 50;
  r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);  // battery rose
  TEST_ASSERT_EQUAL_INT((int)FsReason::Battery, (int)r.trippedReason);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Ota, (int)f.reason());
  TEST_ASSERT_EQUAL_INT((int)Err::BusyOta, (int)f.allowMotion());
  in.otaActive = false;  // failed upload, lock lifted
  in.nowMs = 100;
  f.update(in);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Battery, (int)f.reason());
  TEST_ASSERT_EQUAL_INT((int)Err::LowBattery, (int)f.allowMotion());
}

static void test_runtime_latch() {
  FailsafeLogic f = make();
  f.onMotionCommand(A, 0);
  Inputs in = at(100);
  in.runLimitHit = true;
  Result r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Runtime, (int)r.trippedReason);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_FALSE(r.hardStop);
  TEST_ASSERT_FALSE(f.hasOwner());
  TEST_ASSERT_EQUAL_INT((int)Err::None, (int)f.allowMotion());  // does not block a restart
  in.runLimitHit = false;
  in.nowMs = 150;
  assertQuiet(f.update(in));
  TEST_ASSERT_TRUE(f.has(FsReason::Runtime));  // latched
  f.onFrame(A, 160);
  TEST_ASSERT_TRUE(f.has(FsReason::Runtime));  // frames do not clear it
  f.clearRuntime();
  TEST_ASSERT_FALSE(f.active());
  // still over the limit after a clear: trips again
  in.runLimitHit = true;
  in.nowMs = 200;
  r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Runtime, (int)r.trippedReason);
}

static void test_priorities() {
  FailsafeLogic f = make();
  f.update(at(0, 1));
  f.onMotionCommand(A, 0);
  Inputs in = at(600, 0);  // timeout and last-client disconnect at once
  in.runLimitHit = true;
  Result r = f.update(in);
  TEST_ASSERT_TRUE(r.tripped);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)r.trippedReason);  // highest of the new ones
  TEST_ASSERT_TRUE(f.has(FsReason::Runtime));
  TEST_ASSERT_TRUE(f.has(FsReason::Disconnect));
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)f.reason());
  in.batteryCrit = true;
  in.runLimitHit = false;
  in.nowMs = 650;
  f.update(in);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Battery, (int)f.reason());
  in.otaActive = true;
  f.update(in);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Ota, (int)f.reason());
  // clearing from the top shows the next one down
  in.otaActive = false;
  in.batteryCrit = false;
  f.update(in);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Disconnect, (int)f.reason());
  f.onFrame(B, 700);
  TEST_ASSERT_EQUAL_INT((int)FsReason::Runtime, (int)f.reason());
  f.clearRuntime();
  TEST_ASSERT_EQUAL_INT((int)FsReason::None, (int)f.reason());
  TEST_ASSERT_FALSE(f.active());
}

static void test_timeout_with_battery_active() {
  // A trip while Battery is active still asks to zero the setpoints.
  FailsafeLogic f = make();
  Inputs in = at(0);
  in.batteryCrit = true;
  f.update(in);
  in.batteryCrit = false;
  in.nowMs = 10;
  f.update(in);
  f.onMotionCommand(A, 20);
  in.batteryCrit = true;
  in.nowMs = 1000;
  Result r = f.update(in);
  TEST_ASSERT_TRUE(r.hardStop);
  TEST_ASSERT_TRUE(r.zeroSetpoints);
  TEST_ASSERT_TRUE(f.has(FsReason::Timeout));
  TEST_ASSERT_EQUAL_INT((int)FsReason::Battery, (int)r.trippedReason);
  TEST_ASSERT_FALSE(f.hasOwner());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_client_ref);
  RUN_TEST(test_idle_never_times_out);
  RUN_TEST(test_owner_timeout);
  RUN_TEST(test_owner_frames_keep_alive);
  RUN_TEST(test_other_clients_do_not_keep_owner_alive);
  RUN_TEST(test_new_owner_takes_over);
  RUN_TEST(test_custom_timeout);
  RUN_TEST(test_timestamps_out_of_order_and_wrap);
  RUN_TEST(test_release_owner_stops_supervision);
  RUN_TEST(test_last_client_disconnect_edge);
  RUN_TEST(test_disconnect_edge_with_owner);
  RUN_TEST(test_owner_disconnect);
  RUN_TEST(test_owner_disconnect_after_release_is_harmless);
  RUN_TEST(test_new_motion_supersedes_pending_owner_loss);
  RUN_TEST(test_battery);
  RUN_TEST(test_ota);
  RUN_TEST(test_runtime_latch);
  RUN_TEST(test_priorities);
  RUN_TEST(test_timeout_with_battery_active);
  return UNITY_END();
}
