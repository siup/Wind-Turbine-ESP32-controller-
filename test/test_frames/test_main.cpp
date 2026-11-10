// Frame builders: exact output for the documented examples, valid JSON for
// everything (parsed back with ArduinoJson), null semantics, BLE size limit,
// telemetry signature.
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "config.h"
#include "frames.h"

using proto::Cmd;
using proto::Err;
using proto::FsReason;
using proto::YawDir;

void setUp() {}
void tearDown() {}

static char buf[4096];

// The snapshot behind the full "state" example in docs/PROTOCOL.md.
static TelemetrySnapshot docSnapshot() {
  TelemetrySnapshot s;
  s.rotorTarget = 55;
  s.rotorActual = 48;
  s.rotorRunS = 12;
  s.yawMoving = YawDir::Left;
  s.yawSpeed = 60;
  s.hasAngle = false;
  s.twist = 0;
  s.bal = -1520;
  s.balMax = 20000;
  s.hasPos = true;
  s.pos = -27;
  s.posEst = true;
  s.hasPosMax = true;
  s.posMax = 360;
  s.sensor = "none";
  s.batMv = 7840;
  s.batPct = 62;
  s.hasRssi = true;
  s.rssi = -58;
  s.ws = 1;
  s.ble = 0;
  s.sta = 1;
  s.cfgRev = 3;
  s.upMs = 12345;
  return s;
}

static JsonDocument parseBack(const char* text, size_t len) {
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, text, len);
  TEST_ASSERT_FALSE_MESSAGE((bool)e, text);
  TEST_ASSERT_TRUE_MESSAGE(doc.is<JsonObject>(), text);
  return doc;
}

static void assertAscii(const char* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)s[i];
    TEST_ASSERT_TRUE(c >= 0x20 && c < 0x7F);
  }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

static void test_state_full_matches_doc() {
  size_t n = buildState(docSnapshot(), false, buf, sizeof(buf));
  const char* want =
      "{\"t\":\"state\",\"v\":1,"
      "\"rotor\":{\"target\":55,\"actual\":48,\"run\":12},"
      "\"yaw\":{\"moving\":\"left\",\"speed\":60,\"angle\":null,\"limitL\":false,\"limitR\":false,"
      "\"twist\":null,\"bal\":-1520,\"balMax\":20000,\"pos\":-27,\"est\":true,\"posMax\":360,"
      "\"goto\":null,\"sensor\":\"none\",\"sensorOk\":true,\"cal\":false},"
      "\"bat\":{\"mv\":7840,\"pct\":62,\"low\":false,\"crit\":false,\"present\":true},"
      "\"link\":{\"rssi\":-58,\"clients\":1,\"ws\":1,\"ble\":0,\"sta\":1},"
      "\"fs\":{\"active\":false,\"reason\":null},"
      "\"ota\":false,\"cfgRev\":3,\"up\":12345}";
  TEST_ASSERT_EQUAL_STRING(want, buf);
  TEST_ASSERT_EQUAL_UINT32(strlen(want), n);
}

static void test_state_brief_matches_doc() {
  size_t n = buildState(docSnapshot(), true, buf, sizeof(buf));
  const char* want =
      "{\"t\":\"state\",\"v\":1,\"rotor\":{\"target\":55,\"actual\":48},"
      "\"yaw\":{\"moving\":\"left\",\"angle\":null},\"bat\":{\"pct\":62,\"crit\":false},"
      "\"fs\":{\"active\":false,\"reason\":null}}";
  TEST_ASSERT_EQUAL_STRING(want, buf);
  TEST_ASSERT_EQUAL_UINT32(strlen(want), n);
}

static void test_state_full_keys_and_values() {
  TelemetrySnapshot s = docSnapshot();
  s.yawMoving = YawDir::Right;
  s.hasAngle = true;
  s.angle = 271;
  s.limitL = true;
  s.twist = -1;
  s.hasPos = false;
  s.posEst = false;
  s.hasPosMax = false;
  s.gotoActive = true;
  s.gotoTarget = 135;
  s.sensor = "as5600";
  s.sensorOk = false;
  s.calActive = true;
  s.batLow = true;
  s.batCrit = true;
  s.batPresent = false;
  s.hasRssi = false;
  s.ws = 2;
  s.ble = 1;
  s.sta = 2;
  s.fsActive = true;
  s.fsReason = FsReason::Battery;
  s.ota = true;
  size_t n = buildState(s, false, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  JsonDocument doc = parseBack(buf, n);

  TEST_ASSERT_EQUAL_STRING("state", doc["t"]);
  TEST_ASSERT_EQUAL_INT(PROTOCOL_VERSION, doc["v"].as<int>());
  TEST_ASSERT_EQUAL_INT(55, doc["rotor"]["target"].as<int>());
  TEST_ASSERT_EQUAL_INT(48, doc["rotor"]["actual"].as<int>());
  TEST_ASSERT_EQUAL_INT(12, doc["rotor"]["run"].as<int>());

  JsonObject yaw = doc["yaw"];
  TEST_ASSERT_EQUAL_STRING("right", yaw["moving"]);
  TEST_ASSERT_EQUAL_INT(60, yaw["speed"].as<int>());
  TEST_ASSERT_EQUAL_INT(271, yaw["angle"].as<int>());
  TEST_ASSERT_TRUE(yaw["limitL"].as<bool>());
  TEST_ASSERT_FALSE(yaw["limitR"].as<bool>());
  TEST_ASSERT_EQUAL_STRING("L", yaw["twist"]);
  TEST_ASSERT_EQUAL_INT(-1520, yaw["bal"].as<int>());
  TEST_ASSERT_EQUAL_INT(20000, yaw["balMax"].as<int>());
  TEST_ASSERT_TRUE(yaw["pos"].isNull());
  TEST_ASSERT_FALSE(yaw["pos"].isUnbound());
  TEST_ASSERT_TRUE(yaw["est"].is<bool>());
  TEST_ASSERT_FALSE(yaw["est"].as<bool>());
  TEST_ASSERT_TRUE(yaw["posMax"].isNull());
  TEST_ASSERT_EQUAL_INT(135, yaw["goto"].as<int>());
  TEST_ASSERT_EQUAL_STRING("as5600", yaw["sensor"]);
  TEST_ASSERT_FALSE(yaw["sensorOk"].as<bool>());
  TEST_ASSERT_TRUE(yaw["cal"].as<bool>());

  JsonObject bat = doc["bat"];
  TEST_ASSERT_EQUAL_INT(7840, bat["mv"].as<int>());
  TEST_ASSERT_EQUAL_INT(62, bat["pct"].as<int>());
  TEST_ASSERT_TRUE(bat["low"].as<bool>());
  TEST_ASSERT_TRUE(bat["crit"].as<bool>());
  TEST_ASSERT_FALSE(bat["present"].as<bool>());

  JsonObject link = doc["link"];
  TEST_ASSERT_TRUE(link["rssi"].isNull());
  TEST_ASSERT_EQUAL_INT(3, link["clients"].as<int>());  // ws + ble
  TEST_ASSERT_EQUAL_INT(2, link["ws"].as<int>());
  TEST_ASSERT_EQUAL_INT(1, link["ble"].as<int>());
  TEST_ASSERT_EQUAL_INT(2, link["sta"].as<int>());

  TEST_ASSERT_TRUE(doc["fs"]["active"].as<bool>());
  TEST_ASSERT_EQUAL_STRING("battery", doc["fs"]["reason"]);
  TEST_ASSERT_TRUE(doc["ota"].as<bool>());
  TEST_ASSERT_EQUAL_INT(3, doc["cfgRev"].as<int>());
  TEST_ASSERT_EQUAL_INT(12345, doc["up"].as<int>());

  // every documented key is present (null counts as present)
  static const char* const kTop[] = {"t", "v", "rotor", "yaw", "bat", "link", "fs", "ota", "cfgRev", "up"};
  static const char* const kYaw[] = {"moving", "speed", "angle", "limitL", "limitR", "twist", "bal",
                                     "balMax", "pos",   "est",   "posMax", "goto",   "sensor",
                                     "sensorOk", "cal"};
  JsonObject root = doc.as<JsonObject>();
  for (const char* k : kTop) TEST_ASSERT_FALSE_MESSAGE(root[k].isUnbound(), k);
  for (const char* k : kYaw) TEST_ASSERT_FALSE_MESSAGE(yaw[k].isUnbound(), k);
  TEST_ASSERT_EQUAL_UINT32(10, root.size());
  TEST_ASSERT_EQUAL_UINT32(15, yaw.size());
  TEST_ASSERT_EQUAL_UINT32(5, bat.size());
  TEST_ASSERT_EQUAL_UINT32(5, link.size());
}

static void test_state_null_semantics() {
  TelemetrySnapshot s;  // defaults: nothing known
  s.sensor = nullptr;   // treated as "none"
  size_t n = buildState(s, false, buf, sizeof(buf));
  JsonDocument doc = parseBack(buf, n);
  JsonObject yaw = doc["yaw"];
  TEST_ASSERT_TRUE(yaw["moving"].isNull());
  TEST_ASSERT_TRUE(yaw["angle"].isNull());
  TEST_ASSERT_TRUE(yaw["twist"].isNull());
  TEST_ASSERT_TRUE(yaw["pos"].isNull());
  TEST_ASSERT_TRUE(yaw["est"].as<bool>());  // posEst default
  TEST_ASSERT_TRUE(yaw["posMax"].isNull());
  TEST_ASSERT_TRUE(yaw["goto"].isNull());
  TEST_ASSERT_EQUAL_STRING("none", yaw["sensor"]);
  TEST_ASSERT_TRUE(doc["link"]["rssi"].isNull());
  TEST_ASSERT_EQUAL_INT(0, doc["link"]["clients"].as<int>());
  TEST_ASSERT_TRUE(doc["fs"]["reason"].isNull());
  TEST_ASSERT_FALSE(doc["fs"]["reason"].isUnbound());

  s.twist = 1;
  s.hasPos = true;
  s.pos = 0;
  s.hasPosMax = true;
  s.posMax = 0;
  s.gotoActive = true;
  s.gotoTarget = 0;
  s.hasAngle = true;
  s.angle = 0;
  s.hasRssi = true;
  s.rssi = 0;
  n = buildState(s, false, buf, sizeof(buf));
  doc = parseBack(buf, n);
  yaw = doc["yaw"];
  TEST_ASSERT_EQUAL_STRING("R", yaw["twist"]);
  TEST_ASSERT_EQUAL_INT(0, yaw["pos"].as<int>());
  TEST_ASSERT_FALSE(yaw["pos"].isNull());
  TEST_ASSERT_FALSE(yaw["posMax"].isNull());
  TEST_ASSERT_FALSE(yaw["goto"].isNull());
  TEST_ASSERT_FALSE(yaw["angle"].isNull());
  TEST_ASSERT_FALSE(doc["link"]["rssi"].isNull());
}

static void test_state_every_reason() {
  struct {
    FsReason r;
    const char* name;
  } k[] = {{FsReason::Runtime, "runtime"},
           {FsReason::Timeout, "timeout"},
           {FsReason::Disconnect, "disconnect"},
           {FsReason::Battery, "battery"},
           {FsReason::Ota, "ota"}};
  for (auto& e : k) {
    TelemetrySnapshot s;
    s.fsActive = true;
    s.fsReason = e.r;
    for (int brief = 0; brief < 2; brief++) {
      size_t n = buildState(s, brief != 0, buf, sizeof(buf));
      JsonDocument doc = parseBack(buf, n);
      TEST_ASSERT_EQUAL_STRING(e.name, doc["fs"]["reason"]);
    }
  }
}

static void test_state_brief_worst_case_fits_ble() {
  size_t worst = 0;
  const YawDir dirs[] = {YawDir::Left, YawDir::Right, YawDir::Stop};
  for (int reason = 0; reason <= (int)FsReason::Ota; reason++) {
    for (YawDir d : dirs) {
      for (int hasAngle = 0; hasAngle < 2; hasAngle++) {
        for (int flags = 0; flags < 4; flags++) {
          TelemetrySnapshot s;
          s.rotorTarget = 100;
          s.rotorActual = 100;
          s.yawMoving = d;
          s.hasAngle = hasAngle != 0;
          s.angle = 359;
          s.batPct = 100;
          s.batCrit = flags & 1;
          s.fsActive = flags & 2;
          s.fsReason = (FsReason)reason;
          // full-frame-only fields at their widest must not leak in
          s.rotorRunS = 0xFFFFFFFFu;
          s.bal = INT32_MIN;
          s.upMs = 0xFFFFFFFFu;
          s.batMv = 0xFFFFFFFFu;
          size_t n = buildState(s, true, buf, sizeof(buf));
          TEST_ASSERT_TRUE(n > 0);
          parseBack(buf, n);
          if (n > worst) worst = n;
        }
      }
    }
  }
  char msg[64];
  snprintf(msg, sizeof(msg), "worst brief state frame: %u bytes", (unsigned)worst);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(worst <= BLE_STATE_MAX_BYTES);
  // and it is built fine into a buffer of exactly that size
  TelemetrySnapshot s;
  s.rotorTarget = s.rotorActual = s.batPct = 100;
  s.yawMoving = YawDir::Right;
  s.fsReason = FsReason::Disconnect;
  char small[BLE_STATE_MAX_BYTES + 1];
  TEST_ASSERT_TRUE(buildState(s, true, small, sizeof(small)) > 0);
}

static void test_state_full_worst_case_size() {
  TelemetrySnapshot s;
  s.rotorTarget = s.rotorActual = 100;
  s.rotorRunS = 0xFFFFFFFFu;
  s.yawMoving = YawDir::Right;
  s.yawSpeed = 100;
  s.hasAngle = true;
  s.angle = -32768;
  s.limitL = s.limitR = false;
  s.twist = 1;
  s.bal = INT32_MIN;
  s.balMax = 0xFFFFFFFFu;
  s.hasPos = true;
  s.pos = INT32_MIN;
  s.posEst = false;
  s.hasPosMax = true;
  s.posMax = INT32_MIN;
  s.gotoActive = true;
  s.gotoTarget = -32768;
  s.sensor = "endstops";
  s.sensorOk = false;
  s.calActive = false;
  s.batMv = 0xFFFFFFFFu;
  s.batPct = 100;
  s.batPresent = false;
  s.hasRssi = true;
  s.rssi = -128;
  s.ws = s.ble = s.sta = 255;
  s.fsActive = false;
  s.fsReason = FsReason::Disconnect;
  s.ota = false;
  s.cfgRev = s.upMs = 0xFFFFFFFFu;
  size_t n = buildState(s, false, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  JsonDocument doc = parseBack(buf, n);
  TEST_ASSERT_EQUAL_INT(510, doc["link"]["clients"].as<int>());
  char msg[64];
  snprintf(msg, sizeof(msg), "worst full state frame: %u bytes", (unsigned)n);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(n < 600);
}

static void test_buffer_too_small() {
  TelemetrySnapshot s = docSnapshot();
  size_t n = buildState(s, false, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  char exact[600];
  TEST_ASSERT_EQUAL_UINT32(n, buildState(s, false, exact, n + 1));  // room for the NUL
  TEST_ASSERT_EQUAL_STRING(buf, exact);
  memset(exact, 'x', sizeof(exact));
  TEST_ASSERT_EQUAL_UINT32(0, buildState(s, false, exact, n));
  TEST_ASSERT_EQUAL_CHAR('\0', exact[0]);
  TEST_ASSERT_EQUAL_CHAR('x', exact[n]);  // never writes past cap
  TEST_ASSERT_EQUAL_UINT32(0, buildState(s, false, exact, 1));
  TEST_ASSERT_EQUAL_UINT32(0, buildState(s, false, nullptr, 0));
  TEST_ASSERT_EQUAL_UINT32(0, buildAck(Cmd::Rotor, false, 0, exact, 5));
  TEST_ASSERT_EQUAL_UINT32(0, buildCfg(Settings{}, 0, false, exact, 10));
}

// ---------------------------------------------------------------------------
// signature
// ---------------------------------------------------------------------------

static void test_signature_ignores_slow_fields() {
  TelemetrySnapshot a = docSnapshot();
  TelemetrySnapshot b = a;
  b.upMs += 5000;
  b.rssi = -80;
  b.hasRssi = !a.hasRssi;
  b.batMv = 7000;
  b.rotorRunS = 999;
  TEST_ASSERT_EQUAL_UINT32(stateSignature(a), stateSignature(b));
}

static void test_signature_sees_state_changes() {
  const TelemetrySnapshot a = docSnapshot();
  const uint32_t sa = stateSignature(a);
  TelemetrySnapshot b;
#define CHANGED(stmt)                                     \
  do {                                                    \
    b = a;                                                \
    stmt;                                                 \
    TEST_ASSERT_TRUE_MESSAGE(sa != stateSignature(b), #stmt);     \
  } while (0)
  CHANGED(b.rotorTarget = 56);
  CHANGED(b.rotorActual = 49);
  CHANGED(b.yawMoving = YawDir::Stop);
  CHANGED(b.yawSpeed = 61);
  CHANGED(b.limitL = true);
  CHANGED(b.limitR = true);
  CHANGED(b.twist = 1);
  CHANGED(b.bal = -1519);
  CHANGED(b.balMax = 0);
  CHANGED(b.hasPos = false);
  CHANGED(b.pos = -28);
  CHANGED(b.posEst = false);
  CHANGED(b.hasPosMax = false);
  CHANGED(b.posMax = 720);
  CHANGED(b.gotoActive = true);
  CHANGED(b.sensor = "endstops");
  CHANGED(b.sensorOk = false);
  CHANGED(b.calActive = true);
  CHANGED(b.batPct = 61);
  CHANGED(b.batLow = true);
  CHANGED(b.batCrit = true);
  CHANGED(b.batPresent = false);
  CHANGED(b.ws = 2);
  CHANGED(b.ble = 1);
  CHANGED(b.sta = 2);
  CHANGED(b.fsActive = true);
  CHANGED(b.fsReason = FsReason::Timeout);
  CHANGED(b.ota = true);
  CHANGED(b.cfgRev = 4);
  CHANGED((b.hasAngle = true, b.angle = 10));
#undef CHANGED
}

static void test_signature_idle_angle_dither() {
  TelemetrySnapshot a;
  a.hasAngle = true;
  a.angle = 100;
  a.hasPos = true;
  a.pos = 100;
  a.posEst = false;
  TelemetrySnapshot b = a;
  b.angle = 101;
  b.pos = 101;
  // idle: a one-degree dither is not worth an extra frame
  TEST_ASSERT_EQUAL_UINT32(stateSignature(a), stateSignature(b));
  b.angle = 103;
  b.pos = 103;
  TEST_ASSERT_TRUE(stateSignature(a) != stateSignature(b));
  // moving: every degree counts
  a.yawMoving = b.yawMoving = YawDir::Right;
  b.angle = 101;
  b.pos = 101;
  TEST_ASSERT_TRUE(stateSignature(a) != stateSignature(b));
  a.yawMoving = b.yawMoving = YawDir::Stop;
  a.gotoActive = b.gotoActive = true;
  TEST_ASSERT_TRUE(stateSignature(a) != stateSignature(b));
}

// ---------------------------------------------------------------------------
// ack / err / pong
// ---------------------------------------------------------------------------

static void test_ack() {
  size_t n = buildAck(Cmd::Rotor, true, 5, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"ack\",\"of\":\"rotor\",\"id\":5}", buf);
  TEST_ASSERT_EQUAL_UINT32(strlen(buf), n);
  buildAck(Cmd::Cfg, false, 5, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"ack\",\"of\":\"cfg\"}", buf);
  buildAck(Cmd::None, true, 4294967295u, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"ack\",\"of\":\"unknown\",\"id\":4294967295}", buf);
}

static void test_err() {
  buildErr(Cmd::Yaw, Err::NoSensor, nullptr, "goto requires AS5600", true, 6, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING(
      "{\"t\":\"err\",\"of\":\"yaw\",\"code\":\"no_sensor\",\"msg\":\"goto requires AS5600\",\"id\":6}",
      buf);
  buildErr(Cmd::Cfg, Err::OutOfRange, "rotorMax", nullptr, false, 0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING(
      "{\"t\":\"err\",\"of\":\"cfg\",\"code\":\"out_of_range\",\"field\":\"rotorMax\",\"msg\":\"value out "
      "of range\"}",
      buf);
  // every code builds valid JSON with the default message
  for (int e = (int)Err::BadJson; e <= (int)Err::Stale; e++) {
    size_t n = buildErr(Cmd::None, (Err)e, nullptr, nullptr, true, 1, buf, sizeof(buf));
    JsonDocument doc = parseBack(buf, n);
    TEST_ASSERT_EQUAL_STRING(proto::errCode((Err)e), doc["code"]);
    TEST_ASSERT_TRUE(strlen(doc["code"].as<const char*>()) > 0);
    TEST_ASSERT_TRUE(strlen(doc["msg"].as<const char*>()) > 0);
    TEST_ASSERT_EQUAL_STRING("unknown", doc["of"]);
  }
}

static void test_pong() {
  buildPong(true, 7, 12345, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"pong\",\"id\":7,\"up\":12345}", buf);
  buildPong(false, 7, 0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"pong\",\"up\":0}", buf);
}

static void test_protocol_names() {
  TEST_ASSERT_EQUAL_STRING("rotor", proto::cmdName(Cmd::Rotor));
  TEST_ASSERT_EQUAL_STRING("reboot", proto::cmdName(Cmd::Reboot));
  TEST_ASSERT_EQUAL_STRING("unknown", proto::cmdName(Cmd::None));
  TEST_ASSERT_EQUAL_STRING("", proto::errCode(Err::None));
  TEST_ASSERT_EQUAL_STRING("bad_json", proto::errCode(Err::BadJson));
  TEST_ASSERT_EQUAL_STRING("twist_limit", proto::errCode(Err::TwistLimit));
  TEST_ASSERT_EQUAL_STRING("busy_ota", proto::errCode(Err::BusyOta));
  TEST_ASSERT_EQUAL_STRING("stale", proto::errCode(Err::Stale));
  TEST_ASSERT_NULL(proto::fsReasonName(FsReason::None));
  TEST_ASSERT_EQUAL_STRING("disconnect", proto::fsReasonName(FsReason::Disconnect));
  TEST_ASSERT_EQUAL_STRING("left", proto::yawDirName(YawDir::Left));
  TEST_ASSERT_EQUAL_STRING("stop", proto::yawDirName(YawDir::Stop));
  TEST_ASSERT_EQUAL_STRING("ble", proto::sourceName(proto::Source::Ble));
  TEST_ASSERT_EQUAL_STRING("none", proto::sourceName(proto::Source::None));
}

// ---------------------------------------------------------------------------
// cfg / info / log / evt
// ---------------------------------------------------------------------------

static void test_cfg_frame() {
  Settings s;
  settingsDefaults(s);
  size_t n = buildCfg(s, 3, false, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_STRING_LEN("{\"t\":\"cfg\",\"rev\":3,\"reboot\":false,\"apPassSet\":true,\"rotorMax\":100,", buf,
                               strlen("{\"t\":\"cfg\",\"rev\":3,\"reboot\":false,\"apPassSet\":true,\"rotorMax\":100,"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"batCal\":1.0,"));
  TEST_ASSERT_NULL(strstr(buf, "apPass\":"));       // secret never serialized
  TEST_ASSERT_NULL(strstr(buf, AP_PASSWORD));
  JsonDocument doc = parseBack(buf, n);
  JsonObject root = doc.as<JsonObject>();
  TEST_ASSERT_EQUAL_STRING("cfg", root["t"]);
  TEST_ASSERT_TRUE(root["apPassSet"].as<bool>());
  TEST_ASSERT_TRUE(root["apPass"].isUnbound());
  TEST_ASSERT_EQUAL_UINT32(4 + kSettingsFieldCount - 1, root.size());
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    JsonVariant v = root[f.key];
    if (f.flags & kFieldSecret) {
      TEST_ASSERT_TRUE_MESSAGE(v.isUnbound(), f.key);
      continue;
    }
    TEST_ASSERT_FALSE_MESSAGE(v.isUnbound(), f.key);
    switch (f.type) {
      case FieldType::Str:
        TEST_ASSERT_EQUAL_STRING(settingsGetStr(s, f), v.as<const char*>());
        break;
      case FieldType::Bool:
        TEST_ASSERT_TRUE_MESSAGE(v.is<bool>(), f.key);
        TEST_ASSERT_TRUE_MESSAGE((settingsGetNumber(s, f) != 0.0) == v.as<bool>(), f.key);
        break;
      default:
        TEST_ASSERT_TRUE_MESSAGE(v.is<double>(), f.key);
        TEST_ASSERT_FLOAT_WITHIN(0.0005f, (float)settingsGetNumber(s, f), v.as<float>());
        break;
    }
  }
}

static void test_cfg_frame_floats_and_flags() {
  Settings s;
  settingsDefaults(s);
  s.batCal = 0.987f;
  buildCfg(s, 0, true, buf, sizeof(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"batCal\":0.987,"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"reboot\":true"));
  s.batCal = 1.25f;
  buildCfg(s, 0, false, buf, sizeof(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"batCal\":1.25,"));
  s.batCal = 1.0004f;
  buildCfg(s, 0, false, buf, sizeof(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"batCal\":1.0,"));
  s.batCal = 0.5f;
  buildCfg(s, 0, false, buf, sizeof(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"batCal\":0.5,"));
  s.apPass[0] = '\0';
  buildCfg(s, 0, false, buf, sizeof(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"apPassSet\":false"));
}

static void test_cfg_frame_worst_case_size() {
  Settings s;
  settingsDefaults(s);
  memset(s.name, 'N', 24);
  s.name[24] = '\0';
  s.yawMaxBal = s.yawMs360 = 600000;
  s.rotorRamp = 10000;
  s.batCal = 1.234f;
  size_t n = buildCfg(s, 0xFFFFFFFFu, true, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  parseBack(buf, n);
  char msg[64];
  snprintf(msg, sizeof(msg), "worst cfg frame: %u bytes", (unsigned)n);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE(n < 640);
}

static void test_info_matches_doc() {
  DeviceInfo d;
  d.fw = "1.0.0";
  d.build = "Nov  9 2026 12:00:00";
  d.name = "Turbina-AB12";
  d.chip = "ESP32";
  d.mac = "24:6F:28:AA:AB:12";
  d.sensor = "none";
  d.wifi = d.ble = d.ota = true;
  d.heap = 151234;
  d.heapMin = 120112;
  d.resetReason = "poweron";
  d.upMs = 12345;
  size_t n = buildInfo(d, buf, sizeof(buf));
  const char* want =
      "{\"t\":\"info\",\"v\":1,\"fw\":\"1.0.0\",\"build\":\"Nov  9 2026 12:00:00\",\"name\":\"Turbina-AB12\","
      "\"chip\":\"ESP32\",\"mac\":\"24:6F:28:AA:AB:12\",\"sensor\":\"none\",\"wifi\":true,\"ble\":true,"
      "\"ota\":true,\"heap\":151234,\"heapMin\":120112,\"reset\":\"poweron\",\"up\":12345}";
  TEST_ASSERT_EQUAL_STRING(want, buf);
  TEST_ASSERT_EQUAL_UINT32(strlen(want), n);

  DeviceInfo empty;
  empty.fw = nullptr;
  empty.sensor = nullptr;
  n = buildInfo(empty, buf, sizeof(buf));
  JsonDocument doc = parseBack(buf, n);
  TEST_ASSERT_EQUAL_STRING("", doc["fw"]);
  TEST_ASSERT_EQUAL_STRING("none", doc["sensor"]);
}

static LogItem item(uint32_t ms, char lvl, const char* code, const char* msg) {
  LogItem it;
  it.ms = ms;
  it.lvl = lvl;
  strncpy(it.code, code, sizeof(it.code) - 1);
  strncpy(it.msg, msg, sizeof(it.msg) - 1);
  return it;
}

static void test_log_and_evt_match_doc() {
  LogItem it = item(5021, 'w', "fs_timeout", "owner ws#3 silent 512 ms");
  buildLog(&it, 1, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING(
      "{\"t\":\"log\",\"items\":[{\"ms\":5021,\"lvl\":\"w\",\"code\":\"fs_timeout\",\"msg\":\"owner ws#3 "
      "silent 512 ms\"}]}",
      buf);
  buildEvt(it, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING(
      "{\"t\":\"evt\",\"ms\":5021,\"lvl\":\"w\",\"code\":\"fs_timeout\",\"msg\":\"owner ws#3 silent 512 ms\"}",
      buf);
  buildLog(nullptr, 0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("{\"t\":\"log\",\"items\":[]}", buf);
}

static void test_log_full_buffer_order_and_size() {
  LogItem items[EVENT_LOG_SIZE];
  for (int i = 0; i < EVENT_LOG_SIZE; i++) {
    items[i].ms = 0xFFFFFFF0u + (uint32_t)i;
    items[i].lvl = 'e';
    memset(items[i].code, 'c', sizeof(items[i].code));  // no terminator: must stay bounded
    memset(items[i].msg, 'm', sizeof(items[i].msg));
  }
  size_t n = buildLog(items, EVENT_LOG_SIZE, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > 0);
  JsonDocument doc = parseBack(buf, n);
  JsonArray arr = doc["items"];
  TEST_ASSERT_EQUAL_UINT32(EVENT_LOG_SIZE, arr.size());
  for (int i = 0; i < EVENT_LOG_SIZE; i++) {
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFF0u + (uint32_t)i, arr[i]["ms"].as<uint32_t>());
    TEST_ASSERT_EQUAL_UINT32(sizeof(LogItem::code), strlen(arr[i]["code"].as<const char*>()));
    TEST_ASSERT_EQUAL_UINT32(sizeof(LogItem::msg), strlen(arr[i]["msg"].as<const char*>()));
  }
  char msg[64];
  snprintf(msg, sizeof(msg), "worst log frame: %u bytes", (unsigned)n);
  TEST_MESSAGE(msg);
}

static void test_string_escaping() {
  LogItem it = item(1, 'i', "x", "a\"b\\c\nd\x01\xC3\xA9");
  size_t n = buildEvt(it, buf, sizeof(buf));
  assertAscii(buf, n);  // valid UTF-8 for a text WebSocket frame
  JsonDocument doc = parseBack(buf, n);
  TEST_ASSERT_EQUAL_STRING("a\"b\\c\nd\x01??", doc["msg"]);
  it.lvl = '"';
  n = buildEvt(it, buf, sizeof(buf));
  doc = parseBack(buf, n);
  TEST_ASSERT_EQUAL_STRING("\"", doc["lvl"]);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_state_full_matches_doc);
  RUN_TEST(test_state_brief_matches_doc);
  RUN_TEST(test_state_full_keys_and_values);
  RUN_TEST(test_state_null_semantics);
  RUN_TEST(test_state_every_reason);
  RUN_TEST(test_state_brief_worst_case_fits_ble);
  RUN_TEST(test_state_full_worst_case_size);
  RUN_TEST(test_buffer_too_small);
  RUN_TEST(test_signature_ignores_slow_fields);
  RUN_TEST(test_signature_sees_state_changes);
  RUN_TEST(test_signature_idle_angle_dither);
  RUN_TEST(test_ack);
  RUN_TEST(test_err);
  RUN_TEST(test_pong);
  RUN_TEST(test_protocol_names);
  RUN_TEST(test_cfg_frame);
  RUN_TEST(test_cfg_frame_floats_and_flags);
  RUN_TEST(test_cfg_frame_worst_case_size);
  RUN_TEST(test_info_matches_doc);
  RUN_TEST(test_log_and_evt_match_doc);
  RUN_TEST(test_log_full_buffer_order_and_size);
  RUN_TEST(test_string_escaping);
  return UNITY_END();
}
