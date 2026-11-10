// Runtime settings in NVS namespace "cfg". Every field of kSettingsFields is
// stored under its JSON key with the NVS type matching its FieldType:
//   U8, Bool -> u8    U16 -> u16    U32 -> u32    Float -> 4-byte blob
//   Str -> string
// Loop task only.

#include <Arduino.h>

#include "settings.h"

#include <Preferences.h>
#include <string.h>

#include "config.h"
#include "event_log.h"
#include "log.h"

namespace {

constexpr const char* kNamespace = "cfg";
// Upper bound for a deferred save while commits keep arriving (a slider held
// for a long time must still reach the flash eventually).
constexpr uint32_t kSaveMaxDeferMs = 3000;

Preferences g_prefs;
bool g_open = false;

Settings g_cur;    // live values
Settings g_saved;  // what NVS holds (per field)
Settings g_boot;   // values the device booted with (rebootPending)
uint32_t g_rev = 1;

bool g_dirty = false;
uint32_t g_firstDirtyMs = 0;
uint32_t g_lastCommitMs = 0;

uint8_t* fieldBytes(Settings& s, const FieldDesc& f) { return reinterpret_cast<uint8_t*>(&s) + f.offset; }

const uint8_t* fieldBytes(const Settings& s, const FieldDesc& f) {
  return reinterpret_cast<const uint8_t*>(&s) + f.offset;
}

void copyField(Settings& dst, const Settings& src, const FieldDesc& f) {
  memcpy(fieldBytes(dst, f), fieldBytes(src, f), f.size);
}

PreferenceType expectedType(FieldType t) {
  switch (t) {
    case FieldType::U8:
    case FieldType::Bool:
      return PT_U8;
    case FieldType::U16:
      return PT_U16;
    case FieldType::U32:
      return PT_U32;
    case FieldType::Float:
      return PT_BLOB;
    case FieldType::Str:
      return PT_STR;
  }
  return PT_INVALID;
}

enum class LoadResult : uint8_t { Ok, Missing, Invalid };

// Read one field into s (which already holds the default). On Invalid, s is
// left unchanged.
LoadResult loadField(Settings& s, const FieldDesc& f) {
  if (!g_prefs.isKey(f.key)) return LoadResult::Missing;
  if (g_prefs.getType(f.key) != expectedType(f.type)) return LoadResult::Invalid;

  bool ok = false;
  switch (f.type) {
    case FieldType::U8:
    case FieldType::Bool:  // stored as u8 (putBool); values above 1 are rejected
      ok = settingsSetNumber(s, f, (double)g_prefs.getUChar(f.key, 0));
      break;
    case FieldType::U16:
      ok = settingsSetNumber(s, f, (double)g_prefs.getUShort(f.key, 0));
      break;
    case FieldType::U32:
      ok = settingsSetNumber(s, f, (double)g_prefs.getUInt(f.key, 0));
      break;
    case FieldType::Float:
      if (g_prefs.getBytesLength(f.key) == sizeof(float)) {
        ok = settingsSetNumber(s, f, (double)g_prefs.getFloat(f.key, NAN));
      }
      break;
    case FieldType::Str: {
      char buf[sizeof(Settings::apPass) + 1];
      static_assert(sizeof(Settings::apPass) >= sizeof(Settings::name), "buffer covers every Str");
      const size_t n = g_prefs.getString(f.key, buf, sizeof(buf));  // incl. NUL, 0 = failed
      if (n > 0) {
        buf[sizeof(buf) - 1] = '\0';
        ok = settingsSetStr(s, f, buf);
      }
      break;
    }
  }
  return ok ? LoadResult::Ok : LoadResult::Invalid;
}

bool storeField(const Settings& s, const FieldDesc& f) {
  switch (f.type) {
    case FieldType::U8:
      return g_prefs.putUChar(f.key, (uint8_t)settingsGetNumber(s, f)) > 0;
    case FieldType::Bool:
      return g_prefs.putBool(f.key, settingsGetNumber(s, f) != 0.0) > 0;
    case FieldType::U16:
      return g_prefs.putUShort(f.key, (uint16_t)settingsGetNumber(s, f)) > 0;
    case FieldType::U32:
      return g_prefs.putUInt(f.key, (uint32_t)settingsGetNumber(s, f)) > 0;
    case FieldType::Float:
      return g_prefs.putFloat(f.key, (float)settingsGetNumber(s, f)) > 0;
    case FieldType::Str:
      return g_prefs.putString(f.key, settingsGetStr(s, f)) > 0;
  }
  return false;
}

// Write every field that differs from what NVS holds.
void save() {
  g_dirty = false;
  if (!g_open) return;
  unsigned written = 0, failed = 0;
  const char* firstFailed = nullptr;
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    if (!settingsFieldDiffers(g_cur, g_saved, i)) continue;
    const FieldDesc& f = kSettingsFields[i];
    if (storeField(g_cur, f)) {
      copyField(g_saved, g_cur, f);
      written++;
    } else {
      // g_saved keeps the old value, so the next save retries this key.
      if (!firstFailed) firstFailed = f.key;
      failed++;
    }
  }
  if (failed) {
    EventLog::add('e', "nvs_write", "cfg: %u keys failed, first %s", failed, firstFailed);
  } else if (written) {
    LOGD("cfg: %u keys saved", written);
  }
}

}  // namespace

namespace SettingsStore {

void begin() {
  settingsDefaults(g_cur);
  g_open = g_prefs.begin(kNamespace, false);
  if (!g_open) {
    EventLog::add('e', "nvs_open", "cfg namespace unavailable, using defaults");
  } else {
    unsigned missing = 0, invalid = 0;
    const char* firstInvalid = nullptr;
    for (size_t i = 0; i < kSettingsFieldCount; i++) {
      const FieldDesc& f = kSettingsFields[i];
      switch (loadField(g_cur, f)) {
        case LoadResult::Ok:
          break;
        case LoadResult::Missing:
          missing++;
          break;
        case LoadResult::Invalid:
          // Drop the bad value: the field reads as missing (default) from now on.
          LOGW("cfg: stored %s invalid, default used", f.key);
          g_prefs.remove(f.key);
          if (!firstInvalid) firstInvalid = f.key;
          invalid++;
          break;
      }
    }
    if (invalid) {
      EventLog::add('w', "cfg_invalid", "%u fields reset to default, first %s", invalid, firstInvalid);
    }
    LOGI("cfg: %u fields stored, %u default", (unsigned)(kSettingsFieldCount - missing - invalid),
         missing + invalid);
  }
  // NVS holds exactly what was loaded (defaults stand in for missing keys).
  g_saved = g_cur;

  // Cross-field rules (batCrit < batLow, batEmpty < batFull): fix the
  // battery thresholds, persisted by the next deferred save.
  int bad = settingsValidate(g_cur);
  if (bad >= 0) {
    const char* key = kSettingsFields[bad].key;
    if (g_cur.batType == BAT_PRESET_9V || g_cur.batType == BAT_PRESET_2S) {
      applyBatteryPreset(g_cur, g_cur.batType);
    } else {
      Settings d;
      settingsDefaults(d);
      g_cur.batLow = d.batLow;
      g_cur.batCrit = d.batCrit;
      g_cur.batEmpty = d.batEmpty;
      g_cur.batFull = d.batFull;
    }
    bad = settingsValidate(g_cur);
    if (bad >= 0) {
      // Should not happen: every field was range-checked on load.
      EventLog::add('e', "cfg_invalid", "%s still invalid, factory defaults", kSettingsFields[bad].key);
      settingsDefaults(g_cur);
    } else {
      EventLog::add('w', "cfg_invalid", "%s inconsistent, battery thresholds reset", key);
    }
    g_dirty = true;
    g_firstDirtyMs = g_lastCommitMs = millis();
  }

  g_boot = g_cur;
  g_rev = 1;
}

const Settings& get() { return g_cur; }

uint32_t revision() { return g_rev; }

void commit(const Settings& next) {
  g_cur = next;
  g_rev++;
  const uint32_t now = millis();
  if (!g_dirty) g_firstDirtyMs = now;
  g_lastCommitMs = now;
  g_dirty = true;
}

void factoryReset() {
  settingsDefaults(g_cur);
  g_rev++;
  if (g_open) {
    if (!g_prefs.clear()) EventLog::add('e', "nvs_write", "cfg clear failed");
  }
  // An empty namespace loads as defaults.
  g_saved = g_cur;
  g_dirty = false;
  EventLog::add('i', "cfg_reset", "factory defaults");
}

bool rebootPending() {
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    if ((kSettingsFields[i].flags & kFieldReboot) && settingsFieldDiffers(g_cur, g_boot, i)) {
      return true;
    }
  }
  return false;
}

void tick(uint32_t nowMs) {
  if (!g_dirty) return;
  if ((uint32_t)(nowMs - g_lastCommitMs) >= SETTINGS_SAVE_DELAY_MS ||
      (uint32_t)(nowMs - g_firstDirtyMs) >= kSaveMaxDeferMs) {
    save();
  }
}

void flush() {
  if (g_dirty) save();
}

}  // namespace SettingsStore
