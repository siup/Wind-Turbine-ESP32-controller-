// Runtime settings: the struct, its defaults and a descriptor table that is
// the single source of truth for NVS keys, JSON keys, types and ranges.
// settings.cpp (NVS), cmd_parser.cpp (cfg patches) and frames.cpp (cfg frame)
// all walk kSettingsFields instead of listing fields by hand.
#pragma once

#include <stddef.h>
#include <stdint.h>

struct Settings {
  // Rotor
  uint8_t rotorMax;      // % ceiling for the rotor speed command (10..100)
  uint8_t rotorMin;      // % dead-band compensation: 1 % maps to this duty (0..60)
  uint16_t rotorRamp;    // ms for the full 0..100 % ramp (50..10000)
  uint8_t rotorDuty;     // hardware PWM ceiling, 0..255 scale (50..255)
  bool rotorRev;         // spin the other way (applied only while stopped)
  uint16_t rotorMaxRun;  // s, 0 = unlimited (0..3600)
  // Yaw
  uint8_t yawSpeed;      // default yaw speed % (20..100)
  uint8_t yawDuty;       // hardware PWM ceiling (50..255)
  bool yawInvert;        // swap left/right (only while stopped; zeroes the balance)
  uint32_t yawMaxBal;    // twist limit, speed-weighted ms; 0 = off (0..600000)
  uint32_t yawMs360;     // speed-weighted ms for one full turn; 0 = unknown (0..600000)
  uint16_t yawMaxDeg;    // AS5600 twist limit in degrees; 0 = off (0..3600)
  bool yawSensRev;       // AS5600 counts the other way (only while stopped)
  // Battery
  uint8_t batType;       // 0 = 9V, 1 = 2S Li-ion, 2 = custom thresholds
  float batCal;          // calibration factor (0.5..1.5)
  uint16_t batLow;       // mV, warning (3000..20000)
  uint16_t batCrit;      // mV, motors stop + start blocked (3000..20000)
  uint16_t batEmpty;     // mV shown as 0 %
  uint16_t batFull;      // mV shown as 100 %
  // Safety
  uint16_t fsTimeout;    // ms of silence from the motion owner (400..1000)
  // Network
  char name[25];         // device / SSID / BLE name; "" = Turbina-XXXX (0..24 chars)
  char apPass[64];       // WPA2 password (8..63 chars), never sent to clients
  uint8_t apChan;        // 1..13
  uint8_t txPower;       // 0.25 dBm units (8..78), snapped to a supported step
  uint8_t bleMode;       // 0 off, 1 auto (only while no WiFi station), 2 always
  bool captive;          // captive-portal DNS
  // Misc
  bool led;              // status LED enabled
};

enum class FieldType : uint8_t { U8, U16, U32, Bool, Float, Str };

enum FieldFlags : uint8_t {
  kFieldReboot = 1,    // takes effect after a restart
  kFieldSecret = 2,    // never serialized (apPass); cfg frame says "apPassSet":true instead
  kFieldIdleOnly = 4,  // may only change while all motors are idle (else err busy)
};

struct FieldDesc {
  const char* key;    // JSON key == NVS key (<= 15 chars)
  FieldType type;
  uint16_t offset;    // offsetof(Settings, field)
  uint16_t size;      // sizeof(field); for Str: capacity incl. NUL
  double min;         // numbers: inclusive range; Str: min length
  double max;         // numbers: inclusive range; Str: max length
  uint8_t flags;      // FieldFlags
};

extern const FieldDesc kSettingsFields[];
extern const size_t kSettingsFieldCount;  // <= 64 (cfg masks are uint64_t)

// Defaults from config.h (battery thresholds from BAT_PRESET).
void settingsDefaults(Settings& s);

// Fill batLow/batCrit/batEmpty/batFull for preset 0 (9V) or 1 (2S).
// Custom (2) leaves them unchanged. Also sets s.batType.
void applyBatteryPreset(Settings& s, uint8_t preset);

// Range check of every field plus cross-field rules (batCrit < batLow,
// batEmpty < batFull). Returns -1 if valid, else the index of the first
// offending field.
int settingsValidate(const Settings& s);

// Lookup by key. Returns nullptr if unknown; *indexOut gets the table index.
const FieldDesc* settingsFindField(const char* key, size_t* indexOut = nullptr);

// Generic access for U8/U16/U32/Bool/Float fields.
double settingsGetNumber(const Settings& s, const FieldDesc& f);
// Range-checked set. Integer fields reject non-integral values. Returns false
// (and leaves s unchanged) if the value is not acceptable.
bool settingsSetNumber(Settings& s, const FieldDesc& f, double v);

// Str fields. Allowed characters: printable ASCII 0x20..0x7E except '"' and
// '\\'. Length bounds from min/max.
const char* settingsGetStr(const Settings& s, const FieldDesc& f);
bool settingsSetStr(Settings& s, const FieldDesc& f, const char* v);

// True if field i differs between a and b.
bool settingsFieldDiffers(const Settings& a, const Settings& b, size_t i);

// Nearest WiFi TX power step supported by the ESP32 (0.25 dBm units):
// 8, 20, 28, 34, 44, 52, 60, 68, 74, 76, 78.
uint8_t snapTxPower(uint8_t qdbm);
