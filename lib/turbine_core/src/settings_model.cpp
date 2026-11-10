#include "settings_model.h"

#include <string.h>

#include <cmath>

#include "config.h"

#define FIELD(name, type, lo, hi, flags) \
  {#name, FieldType::type, (uint16_t)offsetof(Settings, name), (uint16_t)sizeof(Settings::name), lo, hi, flags}

// Order follows the struct. Ranges and flags: docs/PROTOCOL.md, cfg table.
const FieldDesc kSettingsFields[] = {
    // Rotor
    FIELD(rotorMax, U8, 10, 100, 0),
    FIELD(rotorMin, U8, 0, 60, 0),
    FIELD(rotorRamp, U16, 50, 10000, 0),
    FIELD(rotorDuty, U8, 50, 255, 0),
    FIELD(rotorRev, Bool, 0, 1, kFieldIdleOnly),
    FIELD(rotorMaxRun, U16, 0, 3600, 0),
    // Yaw
    FIELD(yawSpeed, U8, 20, 100, 0),
    FIELD(yawDuty, U8, 50, 255, 0),
    FIELD(yawInvert, Bool, 0, 1, kFieldIdleOnly),
    FIELD(yawMaxBal, U32, 0, 600000, 0),
    FIELD(yawMs360, U32, 0, 600000, 0),
    FIELD(yawMaxDeg, U16, 0, 3600, 0),
    FIELD(yawSensRev, Bool, 0, 1, kFieldIdleOnly),
    // Battery
    FIELD(batType, U8, 0, 2, 0),
    FIELD(batCal, Float, 0.5, 1.5, 0),
    FIELD(batLow, U16, 3000, 20000, 0),
    FIELD(batCrit, U16, 3000, 20000, 0),
    FIELD(batEmpty, U16, 3000, 20000, 0),
    FIELD(batFull, U16, 3000, 20000, 0),
    // Safety
    FIELD(fsTimeout, U16, FS_TIMEOUT_MIN_MS, FS_TIMEOUT_MAX_MS, 0),
    // Network
    FIELD(name, Str, 0, 24, kFieldReboot),
    FIELD(apPass, Str, 8, 63, kFieldReboot | kFieldSecret),
    FIELD(apChan, U8, 1, 13, kFieldReboot),
    FIELD(txPower, U8, 8, 78, 0),
    FIELD(bleMode, U8, 0, 2, 0),
    FIELD(captive, Bool, 0, 1, kFieldReboot),
    // Misc
    FIELD(led, Bool, 0, 1, 0),
};

#undef FIELD

const size_t kSettingsFieldCount = sizeof(kSettingsFields) / sizeof(kSettingsFields[0]);

static_assert(sizeof(kSettingsFields) / sizeof(kSettingsFields[0]) <= 64,
              "cfg masks are uint64_t");

namespace {

uint8_t* fieldPtr(Settings& s, const FieldDesc& f) {
  return reinterpret_cast<uint8_t*>(&s) + f.offset;
}

const uint8_t* fieldPtr(const Settings& s, const FieldDesc& f) {
  return reinterpret_cast<const uint8_t*>(&s) + f.offset;
}

bool isNumeric(FieldType t) { return t != FieldType::Str; }

bool isIntegral(FieldType t) {
  return t == FieldType::U8 || t == FieldType::U16 || t == FieldType::U32 || t == FieldType::Bool;
}

// Same rule as the parser: printable ASCII without '"' and '\'.
bool strAcceptable(const FieldDesc& f, const char* v, size_t n) {
  if (n < (size_t)f.min || n > (size_t)f.max || n + 1 > f.size) return false;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)v[i];
    if (c < 0x20 || c > 0x7E || c == '"' || c == '\\') return false;
  }
  return true;
}

bool numberAcceptable(const FieldDesc& f, double v) {
  if (!isNumeric(f.type)) return false;
  if (!std::isfinite(v)) return false;
  if (v < f.min || v > f.max) return false;
  if (isIntegral(f.type) && v != std::floor(v)) return false;
  return true;
}

}  // namespace

void applyBatteryPreset(Settings& s, uint8_t preset) {
  if (preset == BAT_PRESET_9V) {
    s.batLow = BAT9V_LOW_MV;
    s.batCrit = BAT9V_CRIT_MV;
    s.batEmpty = BAT9V_EMPTY_MV;
    s.batFull = BAT9V_FULL_MV;
  } else if (preset == BAT_PRESET_2S) {
    s.batLow = BAT2S_LOW_MV;
    s.batCrit = BAT2S_CRIT_MV;
    s.batEmpty = BAT2S_EMPTY_MV;
    s.batFull = BAT2S_FULL_MV;
  }
  s.batType = preset;
}

void settingsDefaults(Settings& s) {
  memset(&s, 0, sizeof(s));  // deterministic padding and string tails
  s.rotorMax = 100;
  s.rotorMin = 0;
  s.rotorRamp = RAMP_FULL_SCALE_MS;
  s.rotorDuty = ROTOR_MAX_DUTY;
  s.rotorRev = false;
  s.rotorMaxRun = ROTOR_MAX_RUN_S;
  s.yawSpeed = YAW_SPEED_PCT;
  s.yawDuty = YAW_MAX_DUTY;
  s.yawInvert = false;
  s.yawMaxBal = YAW_MAX_BALANCE_MS;
  s.yawMs360 = 0;
  s.yawMaxDeg = 0;
  s.yawSensRev = false;
  // A custom default preset starts from the 9V thresholds.
  applyBatteryPreset(s, BAT_PRESET_9V);
  applyBatteryPreset(s, BAT_PRESET);
  s.batCal = BAT_CAL;
  s.fsTimeout = FS_TIMEOUT_MS;
  s.name[0] = '\0';
  static_assert(sizeof(AP_PASSWORD) <= sizeof(s.apPass), "AP_PASSWORD too long");
  memcpy(s.apPass, AP_PASSWORD, sizeof(AP_PASSWORD));
  s.apChan = AP_CHANNEL;
  s.txPower = WIFI_TX_QDBM;
  s.bleMode = BLE_MODE_DEFAULT;
  s.captive = false;
  s.led = true;
}

int settingsValidate(const Settings& s) {
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    if (f.type == FieldType::Str) {
      const char* v = reinterpret_cast<const char*>(fieldPtr(s, f));
      const void* nul = memchr(v, '\0', f.size);
      if (!nul) return (int)i;
      if (!strAcceptable(f, v, (size_t)((const char*)nul - v))) return (int)i;
    } else if (f.type == FieldType::Bool) {
      uint8_t raw;
      memcpy(&raw, fieldPtr(s, f), 1);
      if (raw > 1) return (int)i;
    } else if (!numberAcceptable(f, settingsGetNumber(s, f))) {
      return (int)i;
    }
  }
  size_t idx = 0;
  if (s.batCrit >= s.batLow) {
    settingsFindField("batCrit", &idx);
    return (int)idx;
  }
  if (s.batEmpty >= s.batFull) {
    settingsFindField("batFull", &idx);
    return (int)idx;
  }
  return -1;
}

const FieldDesc* settingsFindField(const char* key, size_t* indexOut) {
  if (!key) return nullptr;
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    if (strcmp(kSettingsFields[i].key, key) == 0) {
      if (indexOut) *indexOut = i;
      return &kSettingsFields[i];
    }
  }
  return nullptr;
}

double settingsGetNumber(const Settings& s, const FieldDesc& f) {
  const uint8_t* p = fieldPtr(s, f);
  switch (f.type) {
    case FieldType::U8: {
      uint8_t v;
      memcpy(&v, p, sizeof(v));
      return v;
    }
    case FieldType::U16: {
      uint16_t v;
      memcpy(&v, p, sizeof(v));
      return v;
    }
    case FieldType::U32: {
      uint32_t v;
      memcpy(&v, p, sizeof(v));
      return v;
    }
    case FieldType::Bool: {
      bool v;
      memcpy(&v, p, sizeof(v));
      return v ? 1.0 : 0.0;
    }
    case FieldType::Float: {
      float v;
      memcpy(&v, p, sizeof(v));
      return v;
    }
    case FieldType::Str:
      break;
  }
  return 0.0;
}

bool settingsSetNumber(Settings& s, const FieldDesc& f, double v) {
  if (!numberAcceptable(f, v)) return false;
  uint8_t* p = fieldPtr(s, f);
  switch (f.type) {
    case FieldType::U8: {
      uint8_t x = (uint8_t)v;
      memcpy(p, &x, sizeof(x));
      return true;
    }
    case FieldType::U16: {
      uint16_t x = (uint16_t)v;
      memcpy(p, &x, sizeof(x));
      return true;
    }
    case FieldType::U32: {
      uint32_t x = (uint32_t)v;
      memcpy(p, &x, sizeof(x));
      return true;
    }
    case FieldType::Bool: {
      bool x = v != 0.0;
      memcpy(p, &x, sizeof(x));
      return true;
    }
    case FieldType::Float: {
      float x = (float)v;
      memcpy(p, &x, sizeof(x));
      return true;
    }
    case FieldType::Str:
      break;
  }
  return false;
}

const char* settingsGetStr(const Settings& s, const FieldDesc& f) {
  if (f.type != FieldType::Str) return "";
  return reinterpret_cast<const char*>(fieldPtr(s, f));
}

bool settingsSetStr(Settings& s, const FieldDesc& f, const char* v) {
  if (f.type != FieldType::Str || !v) return false;
  size_t n = strnlen(v, f.size);
  if (!strAcceptable(f, v, n)) return false;
  char* dst = reinterpret_cast<char*>(fieldPtr(s, f));
  memset(dst, 0, f.size);
  memcpy(dst, v, n);
  return true;
}

bool settingsFieldDiffers(const Settings& a, const Settings& b, size_t i) {
  if (i >= kSettingsFieldCount) return false;
  const FieldDesc& f = kSettingsFields[i];
  if (f.type == FieldType::Str) {
    return strncmp(settingsGetStr(a, f), settingsGetStr(b, f), f.size) != 0;
  }
  if (f.type == FieldType::Float) {
    return settingsGetNumber(a, f) != settingsGetNumber(b, f);
  }
  return memcmp(fieldPtr(a, f), fieldPtr(b, f), f.size) != 0;
}

uint8_t snapTxPower(uint8_t qdbm) {
  static const uint8_t kSteps[] = {8, 20, 28, 34, 44, 52, 60, 68, 74, 76, 78};
  uint8_t best = kSteps[0];
  int bestDist = 1000;
  for (uint8_t step : kSteps) {
    int d = (int)qdbm - (int)step;
    if (d < 0) d = -d;
    if (d < bestDist) {  // strict: a tie keeps the lower (weaker) step
      bestDist = d;
      best = step;
    }
  }
  return best;
}
