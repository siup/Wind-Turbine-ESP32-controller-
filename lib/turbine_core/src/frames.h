// Outbound frame builders (ESP -> client). Pure functions writing compact,
// NUL-terminated JSON into caller buffers. Each returns the length written
// (excluding NUL) or 0 if the frame did not fit in `cap`.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"
#include "settings_model.h"

// Everything a "state" frame needs, gathered by commands.cpp from the modules.
struct TelemetrySnapshot {
  // rotor
  uint8_t rotorTarget = 0;  // % after the rotorMax clamp
  uint8_t rotorActual = 0;  // % ramp output
  uint32_t rotorRunS = 0;   // seconds the rotor has been running continuously

  // yaw
  proto::YawDir yawMoving = proto::YawDir::Stop;  // null on the wire when Stop
  uint8_t yawSpeed = 0;                           // % of the current/last move
  bool hasAngle = false;    // AS5600 present and magnet OK
  int16_t angle = 0;        // 0..359
  bool limitL = false;      // left blocked (endstop or twist limit)
  bool limitR = false;      // right blocked
  int8_t twist = 0;         // -1 / +1 = twist limit reached on that side, 0 = no
  int32_t bal = 0;          // speed-weighted ms balance, + = right
  uint32_t balMax = 0;      // yawMaxBal (0 = off)
  bool hasPos = false;      // continuous position known
  int32_t pos = 0;          // degrees, continuous (may exceed +-360)
  bool posEst = true;       // pos is an estimate from time (not AS5600)
  bool hasPosMax = false;
  int32_t posMax = 0;       // twist limit in degrees
  bool gotoActive = false;
  int16_t gotoTarget = 0;
  const char* sensor = "none";  // "none" | "endstops" | "as5600"
  bool sensorOk = true;
  bool calActive = false;   // ms-per-360 measurement running

  // battery
  uint32_t batMv = 0;
  uint8_t batPct = 0;
  bool batLow = false;
  bool batCrit = false;
  bool batPresent = true;

  // link
  bool hasRssi = false;
  int8_t rssi = 0;
  uint8_t ws = 0;   // WebSocket clients
  uint8_t ble = 0;  // BLE centrals
  uint8_t sta = 0;  // WiFi stations associated

  // failsafe / system
  bool fsActive = false;
  proto::FsReason fsReason = proto::FsReason::None;
  bool ota = false;
  uint32_t cfgRev = 0;
  uint32_t upMs = 0;
};

struct DeviceInfo {
  const char* fw = "";
  const char* build = "";        // __DATE__ " " __TIME__
  const char* name = "";
  const char* chip = "";         // BOARD_NAME
  const char* mac = "";
  const char* sensor = "none";
  const char* resetReason = "";
  bool wifi = false;
  bool ble = false;
  bool ota = false;
  uint32_t heap = 0;
  uint32_t heapMin = 0;
  uint32_t upMs = 0;
};

struct LogItem {
  uint32_t ms = 0;     // millis() when it happened
  char lvl = 'i';      // 'e' error, 'w' warning, 'i' info
  char code[16] = {};  // machine code, e.g. "fs_timeout"
  char msg[48] = {};   // short English detail
};

// {"t":"state","v":1,...}. brief = BLE variant (<= BLE_STATE_MAX_BYTES):
// rotor.target, rotor.actual, yaw.moving, yaw.angle, bat.pct, bat.crit,
// fs.active, fs.reason.
size_t buildState(const TelemetrySnapshot& s, bool brief, char* out, size_t cap);

// Hash of the fields that matter for "send immediately on change".
// Excludes upMs, rssi, batMv and rotorRunS so idle telemetry stays at the
// periodic rate.
uint32_t stateSignature(const TelemetrySnapshot& s);

size_t buildAck(proto::Cmd of, bool hasId, uint32_t id, char* out, size_t cap);
// msg nullptr -> proto::errMessage(e). field nullptr -> omitted.
size_t buildErr(proto::Cmd of, proto::Err e, const char* field, const char* msg,
                bool hasId, uint32_t id, char* out, size_t cap);
size_t buildPong(bool hasId, uint32_t id, uint32_t upMs, char* out, size_t cap);
// {"t":"cfg","rev":n,"reboot":bool,"apPassSet":true,<every non-secret field>}
size_t buildCfg(const Settings& s, uint32_t rev, bool rebootPending, char* out, size_t cap);
size_t buildInfo(const DeviceInfo& d, char* out, size_t cap);
// {"t":"log","items":[{"ms":..,"lvl":"w","code":"..","msg":".."},...]} oldest first
size_t buildLog(const LogItem* items, size_t n, char* out, size_t cap);
// {"t":"evt","ms":..,"lvl":"w","code":"..","msg":".."}
size_t buildEvt(const LogItem& item, char* out, size_t cap);
