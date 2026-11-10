// StopSeqGuard: HTTP motion frames older than the client's last stop of the
// same axis are refused; everything else passes.
#include <string.h>
#include <unity.h>

#include "cmd_parser.h"
#include "stop_seq_guard.h"

void setUp() {}
void tearDown() {}

static const uint32_t kA = 0x0204A8C0;  // 192.168.4.2
static const uint32_t kB = 0x0304A8C0;  // 192.168.4.3

static Command cmd(const char* json) {
  Command c;
  ParseResult r = parseCommand(json, strlen(json), c);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, (int)r.err, json);
  return c;
}

static bool ok(StopSeqGuard& g, uint32_t client, const char* json, uint32_t now = 1000) {
  return g.accept(client, cmd(json), now);
}

static void test_without_seq_everything_passes() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"stop\"}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":0,\"seq\":100}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30}"));  // no seq: unchecked
}

static void test_motion_older_than_stop_is_refused() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":0,\"seq\":200}"));
  // START sent before the STOP, delivered after it
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30,\"seq\":199}"));
  // a new START after the STOP runs
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30,\"seq\":201}"));
  // brake counts as a rotor stop too
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":0,\"brake\":true,\"seq\":300}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":55,\"seq\":250}"));
}

static void test_axes_are_separate() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"yaw\",\"dir\":\"stop\",\"seq\":500}"));
  // a yaw stop does not refuse an older rotor frame
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":40,\"seq\":400}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"yaw\",\"dir\":\"left\",\"seq\":499}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"yaw\",\"goto\":90,\"seq\":450}"));
  // hold never restarts anything: unchecked
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"yaw\",\"hold\":\"left\",\"seq\":10}"));
  // a rotor stop does not refuse an older yaw frame
  TEST_ASSERT_TRUE(ok(g, kB, "{\"t\":\"rotor\",\"speed\":0,\"seq\":900}"));
  TEST_ASSERT_TRUE(ok(g, kB, "{\"t\":\"yaw\",\"dir\":\"right\",\"seq\":800}"));
}

static void test_global_stop_covers_both_axes() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"stop\",\"seq\":1000}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30,\"seq\":999}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"yaw\",\"dir\":\"right\",\"seq\":998}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"yaw\",\"dir\":\"right\",\"seq\":1001}"));
  // other commands are never refused
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"cfg\",\"rotorMax\":50,\"seq\":1}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"yaw\",\"zero\":true,\"seq\":1}"));
}

static void test_clients_are_separate() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"stop\",\"seq\":5000}"));
  TEST_ASSERT_TRUE(ok(g, kB, "{\"t\":\"rotor\",\"speed\":30,\"seq\":10}"));
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30,\"seq\":10}"));
}

static void test_late_stop_keeps_the_newest() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":0,\"seq\":300}"));
  TEST_ASSERT_TRUE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":0,\"seq\":100}"));  // an older stop arrives late
  TEST_ASSERT_FALSE(ok(g, kA, "{\"t\":\"rotor\",\"speed\":30,\"seq\":200}"));
}

static void test_window_and_wrap() {
  StopSeqGuard g;
  TEST_ASSERT_TRUE(g.accept(kA, cmd("{\"t\":\"stop\",\"seq\":4294967290}"), 1000));
  // the counter wrapped: 5 is newer than 4294967290
  TEST_ASSERT_TRUE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":5}"), 1100));
  TEST_ASSERT_FALSE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":4294967280}"), 1100));
  TEST_ASSERT_FALSE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":4294967280}"),
                             1000 + StopSeqGuard::kWindowMs - 1));
  // after the window the stop is forgotten (clock jumped back, IP reused)
  TEST_ASSERT_TRUE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":4294967280}"),
                            1000 + StopSeqGuard::kWindowMs));
  // an expired marker is replaced by an older new stop
  TEST_ASSERT_TRUE(g.accept(kA, cmd("{\"t\":\"stop\",\"seq\":50}"), 1000 + StopSeqGuard::kWindowMs + 5));
  TEST_ASSERT_FALSE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":49}"),
                             1000 + StopSeqGuard::kWindowMs + 6));
  TEST_ASSERT_TRUE(g.accept(kA, cmd("{\"t\":\"rotor\",\"speed\":30,\"seq\":51}"),
                            1000 + StopSeqGuard::kWindowMs + 6));
}

static void test_lru_replacement() {
  StopSeqGuard g;
  for (uint32_t i = 0; i < StopSeqGuard::kSlots; i++) {
    TEST_ASSERT_TRUE(g.accept(100 + i, cmd("{\"t\":\"stop\",\"seq\":1000}"), 10 + i));
  }
  // client 100 is refreshed, so 101 is the oldest and gets replaced
  TEST_ASSERT_TRUE(g.accept(100, cmd("{\"t\":\"stop\",\"seq\":1001}"), 50));
  TEST_ASSERT_TRUE(g.accept(999, cmd("{\"t\":\"stop\",\"seq\":1000}"), 60));
  TEST_ASSERT_TRUE(g.accept(101, cmd("{\"t\":\"rotor\",\"speed\":9,\"seq\":1}"), 70));
  TEST_ASSERT_FALSE(g.accept(100, cmd("{\"t\":\"rotor\",\"speed\":9,\"seq\":1}"), 70));
  TEST_ASSERT_FALSE(g.accept(102, cmd("{\"t\":\"rotor\",\"speed\":9,\"seq\":1}"), 70));
  TEST_ASSERT_FALSE(g.accept(999, cmd("{\"t\":\"rotor\",\"speed\":9,\"seq\":1}"), 70));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_without_seq_everything_passes);
  RUN_TEST(test_motion_older_than_stop_is_refused);
  RUN_TEST(test_axes_are_separate);
  RUN_TEST(test_global_stop_covers_both_axes);
  RUN_TEST(test_clients_are_separate);
  RUN_TEST(test_late_stop_keeps_the_newest);
  RUN_TEST(test_window_and_wrap);
  RUN_TEST(test_lru_replacement);
  return UNITY_END();
}
