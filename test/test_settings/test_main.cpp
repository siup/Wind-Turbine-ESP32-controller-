// Settings model: descriptor table, defaults, validation, typed access.
#include <string.h>
#include <unity.h>

#include <cmath>

#include "config.h"
#include "settings_model.h"

void setUp() {}
void tearDown() {}

static const FieldDesc& field(const char* key) {
  const FieldDesc* f = settingsFindField(key);
  TEST_ASSERT_NOT_NULL_MESSAGE(f, key);
  return *f;
}

static Settings defaults() {
  Settings s;
  settingsDefaults(s);
  return s;
}

static void test_table_matches_struct() {
  TEST_ASSERT_EQUAL_UINT32(27, kSettingsFieldCount);
  TEST_ASSERT_TRUE(kSettingsFieldCount <= 64);
  static const char* const kOrder[] = {
      "rotorMax", "rotorMin",  "rotorRamp",  "rotorDuty", "rotorRev", "rotorMaxRun", "yawSpeed",
      "yawDuty",  "yawInvert", "yawMaxBal",  "yawMs360",  "yawMaxDeg", "yawSensRev", "batType",
      "batCal",   "batLow",    "batCrit",    "batEmpty",  "batFull",  "fsTimeout",   "name",
      "apPass",   "apChan",    "txPower",    "bleMode",   "captive",  "led"};
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    TEST_ASSERT_EQUAL_STRING(kOrder[i], f.key);
    TEST_ASSERT_TRUE_MESSAGE(strlen(f.key) <= 15, f.key);  // NVS key limit
    if (i > 0) TEST_ASSERT_TRUE_MESSAGE(f.offset > kSettingsFields[i - 1].offset, f.key);
    TEST_ASSERT_TRUE(f.offset + f.size <= sizeof(Settings));
    switch (f.type) {
      case FieldType::U8:
      case FieldType::Bool:
        TEST_ASSERT_EQUAL_UINT16(1, f.size);
        break;
      case FieldType::U16:
        TEST_ASSERT_EQUAL_UINT16(2, f.size);
        break;
      case FieldType::U32:
      case FieldType::Float:
        TEST_ASSERT_EQUAL_UINT16(4, f.size);
        break;
      case FieldType::Str:
        TEST_ASSERT_TRUE(f.max + 1 <= f.size);
        break;
    }
    for (size_t j = 0; j < i; j++) TEST_ASSERT_TRUE(strcmp(f.key, kSettingsFields[j].key) != 0);
  }
  TEST_ASSERT_EQUAL_UINT16(offsetof(Settings, batCal), field("batCal").offset);
  TEST_ASSERT_EQUAL_UINT16(offsetof(Settings, apPass), field("apPass").offset);
  TEST_ASSERT_EQUAL_UINT16(sizeof(((Settings*)nullptr)->name), field("name").size);
}

static void test_flags() {
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    uint8_t want = 0;
    if (!strcmp(f.key, "rotorRev") || !strcmp(f.key, "yawInvert") || !strcmp(f.key, "yawSensRev"))
      want = kFieldIdleOnly;
    if (!strcmp(f.key, "name") || !strcmp(f.key, "apChan") || !strcmp(f.key, "captive"))
      want = kFieldReboot;
    if (!strcmp(f.key, "apPass")) want = kFieldReboot | kFieldSecret;
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(want, f.flags, f.key);
  }
}

static void test_ranges_and_types() {
  struct {
    const char* key;
    FieldType type;
    double lo, hi;
  } want[] = {
      {"rotorMax", FieldType::U8, 10, 100},       {"rotorMin", FieldType::U8, 0, 60},
      {"rotorRamp", FieldType::U16, 50, 10000},   {"rotorDuty", FieldType::U8, 50, 255},
      {"rotorRev", FieldType::Bool, 0, 1},        {"rotorMaxRun", FieldType::U16, 0, 3600},
      {"yawSpeed", FieldType::U8, 20, 100},       {"yawDuty", FieldType::U8, 50, 255},
      {"yawInvert", FieldType::Bool, 0, 1},       {"yawMaxBal", FieldType::U32, 0, 600000},
      {"yawMs360", FieldType::U32, 0, 600000},    {"yawMaxDeg", FieldType::U16, 0, 3600},
      {"yawSensRev", FieldType::Bool, 0, 1},      {"batType", FieldType::U8, 0, 2},
      {"batCal", FieldType::Float, 0.5, 1.5},     {"batLow", FieldType::U16, 3000, 20000},
      {"batCrit", FieldType::U16, 3000, 20000},   {"batEmpty", FieldType::U16, 3000, 20000},
      {"batFull", FieldType::U16, 3000, 20000},   {"fsTimeout", FieldType::U16, 400, 1000},
      {"name", FieldType::Str, 0, 24},            {"apPass", FieldType::Str, 8, 63},
      {"apChan", FieldType::U8, 1, 13},           {"txPower", FieldType::U8, 8, 78},
      {"bleMode", FieldType::U8, 0, 2},           {"captive", FieldType::Bool, 0, 1},
      {"led", FieldType::Bool, 0, 1},
  };
  for (auto& w : want) {
    const FieldDesc& f = field(w.key);
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)w.type, (int)f.type, w.key);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE((float)w.lo, (float)f.min, w.key);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE((float)w.hi, (float)f.max, w.key);
  }
}

static void test_defaults() {
  Settings s = defaults();
  TEST_ASSERT_EQUAL_UINT8(100, s.rotorMax);
  TEST_ASSERT_EQUAL_UINT8(0, s.rotorMin);
  TEST_ASSERT_EQUAL_UINT16(RAMP_FULL_SCALE_MS, s.rotorRamp);
  TEST_ASSERT_EQUAL_UINT8(ROTOR_MAX_DUTY, s.rotorDuty);
  TEST_ASSERT_FALSE(s.rotorRev);
  TEST_ASSERT_EQUAL_UINT16(ROTOR_MAX_RUN_S, s.rotorMaxRun);
  TEST_ASSERT_EQUAL_UINT8(YAW_SPEED_PCT, s.yawSpeed);
  TEST_ASSERT_EQUAL_UINT8(YAW_MAX_DUTY, s.yawDuty);
  TEST_ASSERT_FALSE(s.yawInvert);
  TEST_ASSERT_EQUAL_UINT32(YAW_MAX_BALANCE_MS, s.yawMaxBal);
  TEST_ASSERT_EQUAL_UINT32(0, s.yawMs360);
  TEST_ASSERT_EQUAL_UINT16(0, s.yawMaxDeg);
  TEST_ASSERT_FALSE(s.yawSensRev);
  TEST_ASSERT_EQUAL_UINT8(BAT_PRESET, s.batType);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, BAT_CAL, s.batCal);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_LOW_MV, s.batLow);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_CRIT_MV, s.batCrit);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_EMPTY_MV, s.batEmpty);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_FULL_MV, s.batFull);
  TEST_ASSERT_EQUAL_UINT16(FS_TIMEOUT_MS, s.fsTimeout);
  TEST_ASSERT_EQUAL_STRING("", s.name);
  TEST_ASSERT_EQUAL_STRING(AP_PASSWORD, s.apPass);
  TEST_ASSERT_EQUAL_UINT8(AP_CHANNEL, s.apChan);
  TEST_ASSERT_EQUAL_UINT8(WIFI_TX_QDBM, s.txPower);
  TEST_ASSERT_EQUAL_UINT8(BLE_MODE_DEFAULT, s.bleMode);
  TEST_ASSERT_FALSE(s.captive);
  TEST_ASSERT_TRUE(s.led);
  TEST_ASSERT_EQUAL_INT(-1, settingsValidate(s));

  // deterministic bytes (padding included), so a memcmp of two defaults is equal
  Settings a, b;
  memset(&a, 0xAA, sizeof(a));
  memset(&b, 0x55, sizeof(b));
  settingsDefaults(a);
  settingsDefaults(b);
  TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof(Settings));
}

static void test_battery_presets() {
  Settings s = defaults();
  applyBatteryPreset(s, BAT_PRESET_2S);
  TEST_ASSERT_EQUAL_UINT8(1, s.batType);
  TEST_ASSERT_EQUAL_UINT16(BAT2S_LOW_MV, s.batLow);
  TEST_ASSERT_EQUAL_UINT16(BAT2S_CRIT_MV, s.batCrit);
  TEST_ASSERT_EQUAL_UINT16(BAT2S_EMPTY_MV, s.batEmpty);
  TEST_ASSERT_EQUAL_UINT16(BAT2S_FULL_MV, s.batFull);
  TEST_ASSERT_EQUAL_INT(-1, settingsValidate(s));
  s.batLow = 7777;
  applyBatteryPreset(s, BAT_PRESET_CUSTOM);
  TEST_ASSERT_EQUAL_UINT8(2, s.batType);
  TEST_ASSERT_EQUAL_UINT16(7777, s.batLow);  // custom keeps the values
  applyBatteryPreset(s, BAT_PRESET_9V);
  TEST_ASSERT_EQUAL_UINT8(0, s.batType);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_LOW_MV, s.batLow);
  TEST_ASSERT_EQUAL_UINT16(BAT9V_FULL_MV, s.batFull);
}

static void test_find_field() {
  size_t idx = 99;
  const FieldDesc* f = settingsFindField("yawSpeed", &idx);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_EQUAL_UINT32(6, idx);
  TEST_ASSERT_EQUAL_PTR(&kSettingsFields[6], f);
  idx = 99;
  TEST_ASSERT_NULL(settingsFindField("nope", &idx));
  TEST_ASSERT_EQUAL_UINT32(99, idx);
  TEST_ASSERT_NULL(settingsFindField(nullptr));
  TEST_ASSERT_NULL(settingsFindField("YawSpeed"));
  TEST_ASSERT_NOT_NULL(settingsFindField("led"));
}

static void test_set_number() {
  Settings s = defaults();
  TEST_ASSERT_TRUE(settingsSetNumber(s, field("rotorMax"), 80));
  TEST_ASSERT_EQUAL_UINT8(80, s.rotorMax);
  TEST_ASSERT_EQUAL_FLOAT(80.0f, (float)settingsGetNumber(s, field("rotorMax")));
  // range and integrality: rejected values leave the field unchanged
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("rotorMax"), 9));
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("rotorMax"), 101));
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("rotorMax"), 50.5));
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("rotorMax"), std::nan("")));
  TEST_ASSERT_EQUAL_UINT8(80, s.rotorMax);
  TEST_ASSERT_TRUE(settingsSetNumber(s, field("rotorMax"), 10));
  TEST_ASSERT_TRUE(settingsSetNumber(s, field("rotorMax"), 100));

  TEST_ASSERT_TRUE(settingsSetNumber(s, field("yawMaxBal"), 600000));
  TEST_ASSERT_EQUAL_UINT32(600000, s.yawMaxBal);
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("yawMaxBal"), 600001));
  TEST_ASSERT_TRUE(settingsSetNumber(s, field("rotorRamp"), 10000));
  TEST_ASSERT_EQUAL_UINT16(10000, s.rotorRamp);

  TEST_ASSERT_TRUE(settingsSetNumber(s, field("batCal"), 0.987));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.987f, s.batCal);
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("batCal"), 0.49));
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("batCal"), 1.51));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.987f, s.batCal);

  TEST_ASSERT_TRUE(settingsSetNumber(s, field("led"), 0));
  TEST_ASSERT_FALSE(s.led);
  TEST_ASSERT_TRUE(settingsSetNumber(s, field("led"), 1));
  TEST_ASSERT_TRUE(s.led);
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("led"), 2));
  TEST_ASSERT_FALSE(settingsSetNumber(s, field("led"), 0.5));

  TEST_ASSERT_FALSE(settingsSetNumber(s, field("name"), 1));  // not numeric
  TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)settingsGetNumber(s, field("name")));
}

static void test_set_str() {
  Settings s = defaults();
  const FieldDesc& name = field("name");
  TEST_ASSERT_TRUE(settingsSetStr(s, name, "Turbina ogrodowa"));
  TEST_ASSERT_EQUAL_STRING("Turbina ogrodowa", settingsGetStr(s, name));
  TEST_ASSERT_TRUE(settingsSetStr(s, name, "abc"));
  // the old tail is cleared
  for (size_t i = 3; i < sizeof(s.name); i++) TEST_ASSERT_EQUAL_CHAR('\0', s.name[i]);
  TEST_ASSERT_TRUE(settingsSetStr(s, name, ""));
  TEST_ASSERT_TRUE(settingsSetStr(s, name, "012345678901234567890123"));   // 24
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "0123456789012345678901234")); // 25
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "a\"b"));
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "a\\b"));
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "a\nb"));
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "Wiatrak\xC5\x82"));  // non-ASCII
  TEST_ASSERT_FALSE(settingsSetStr(s, name, "a\x7F"));
  TEST_ASSERT_FALSE(settingsSetStr(s, name, nullptr));
  TEST_ASSERT_EQUAL_STRING("012345678901234567890123", s.name);
  TEST_ASSERT_TRUE(settingsSetStr(s, name, " !#$%&'()*+,-./:;<=>?@["));
  TEST_ASSERT_TRUE(settingsSetStr(s, name, "]^_`{|}~"));

  const FieldDesc& pass = field("apPass");
  TEST_ASSERT_FALSE(settingsSetStr(s, pass, "1234567"));
  TEST_ASSERT_TRUE(settingsSetStr(s, pass, "12345678"));
  char longPass[65];
  memset(longPass, 'p', 64);
  longPass[64] = '\0';
  TEST_ASSERT_FALSE(settingsSetStr(s, pass, longPass));  // 64
  longPass[63] = '\0';
  TEST_ASSERT_TRUE(settingsSetStr(s, pass, longPass));  // 63
  TEST_ASSERT_EQUAL_UINT32(63, strlen(s.apPass));

  TEST_ASSERT_FALSE(settingsSetStr(s, field("rotorMax"), "80"));  // not a string field
  TEST_ASSERT_EQUAL_STRING("", settingsGetStr(s, field("rotorMax")));
}

static void test_validate() {
  Settings s = defaults();
  TEST_ASSERT_EQUAL_INT(-1, settingsValidate(s));
  size_t idx = 0;

  s = defaults();
  s.rotorMax = 5;
  settingsFindField("rotorMax", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));

  s = defaults();
  s.fsTimeout = 2000;
  settingsFindField("fsTimeout", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));

  s = defaults();
  s.batCal = 3.0f;
  settingsFindField("batCal", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));

  s = defaults();
  s.apPass[0] = '\0';  // shorter than 8
  settingsFindField("apPass", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));

  s = defaults();
  memset(s.name, 'x', sizeof(s.name));  // no terminator
  settingsFindField("name", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));

  // cross-field rules
  s = defaults();
  s.batCrit = s.batLow;
  settingsFindField("batCrit", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));
  s = defaults();
  s.batEmpty = s.batFull;
  settingsFindField("batFull", &idx);
  TEST_ASSERT_EQUAL_INT((int)idx, settingsValidate(s));
  s = defaults();
  s.batLow = 6201;  // crit 6200 < low 6201: fine
  TEST_ASSERT_EQUAL_INT(-1, settingsValidate(s));
}

static void test_field_differs() {
  Settings a = defaults();
  Settings b = defaults();
  for (size_t i = 0; i < kSettingsFieldCount; i++) TEST_ASSERT_FALSE(settingsFieldDiffers(a, b, i));
  size_t iMax = 0, iName = 0, iCal = 0, iLed = 0;
  settingsFindField("rotorMax", &iMax);
  settingsFindField("name", &iName);
  settingsFindField("batCal", &iCal);
  settingsFindField("led", &iLed);
  b.rotorMax = 50;
  b.batCal = 1.1f;
  b.led = false;
  strcpy(b.name, "X");
  TEST_ASSERT_TRUE(settingsFieldDiffers(a, b, iMax));
  TEST_ASSERT_TRUE(settingsFieldDiffers(a, b, iName));
  TEST_ASSERT_TRUE(settingsFieldDiffers(a, b, iCal));
  TEST_ASSERT_TRUE(settingsFieldDiffers(a, b, iLed));
  int n = 0;
  for (size_t i = 0; i < kSettingsFieldCount; i++) n += settingsFieldDiffers(a, b, i);
  TEST_ASSERT_EQUAL_INT(4, n);
  // garbage after the terminator does not count
  Settings c = defaults();
  c.name[5] = 'z';
  TEST_ASSERT_FALSE(settingsFieldDiffers(a, c, iName));
  TEST_ASSERT_FALSE(settingsFieldDiffers(a, b, kSettingsFieldCount));
}

static void test_snap_tx_power() {
  TEST_ASSERT_EQUAL_UINT8(8, snapTxPower(0));
  TEST_ASSERT_EQUAL_UINT8(8, snapTxPower(8));
  TEST_ASSERT_EQUAL_UINT8(8, snapTxPower(13));
  TEST_ASSERT_EQUAL_UINT8(8, snapTxPower(14));   // tie: the weaker step
  TEST_ASSERT_EQUAL_UINT8(20, snapTxPower(15));
  TEST_ASSERT_EQUAL_UINT8(20, snapTxPower(24));  // tie: the weaker step
  TEST_ASSERT_EQUAL_UINT8(28, snapTxPower(25));
  TEST_ASSERT_EQUAL_UINT8(34, snapTxPower(34));
  TEST_ASSERT_EQUAL_UINT8(34, snapTxPower(35));
  TEST_ASSERT_EQUAL_UINT8(44, snapTxPower(40));
  TEST_ASSERT_EQUAL_UINT8(76, snapTxPower(77));
  TEST_ASSERT_EQUAL_UINT8(78, snapTxPower(78));
  TEST_ASSERT_EQUAL_UINT8(78, snapTxPower(255));
  static const uint8_t kSteps[] = {8, 20, 28, 34, 44, 52, 60, 68, 74, 76, 78};
  for (uint8_t v : kSteps) TEST_ASSERT_EQUAL_UINT8(v, snapTxPower(v));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_table_matches_struct);
  RUN_TEST(test_flags);
  RUN_TEST(test_ranges_and_types);
  RUN_TEST(test_defaults);
  RUN_TEST(test_battery_presets);
  RUN_TEST(test_find_field);
  RUN_TEST(test_set_number);
  RUN_TEST(test_set_str);
  RUN_TEST(test_validate);
  RUN_TEST(test_field_differs);
  RUN_TEST(test_snap_tx_power);
  return UNITY_END();
}
