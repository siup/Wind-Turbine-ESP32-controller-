// Protocol v1 vocabulary shared by firmware, parser and frame builders.
// The wire format is documented in docs/PROTOCOL.md.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace proto {

constexpr int kVersion = 1;
constexpr size_t kMaxCmdLen = 256;  // longest accepted inbound frame, bytes

// Inbound command types ("t" field).
enum class Cmd : uint8_t { None, Rotor, Yaw, Stop, Ping, Cfg, Info, Log, Reboot };

// Yaw direction. Right = +1 = increasing angle (clockwise seen from above).
enum class YawDir : int8_t { Left = -1, Stop = 0, Right = 1 };

// What a {"t":"yaw"} frame asks for.
enum class YawAction : uint8_t {
  Move,       // {"dir":"left|right|stop"} starts (or stops) a hold-to-run move
  Hold,       // {"hold":"left|right"} keeps an already running move alive (deadman)
  Goto,       // {"goto":deg} absolute angle, needs AS5600
  Zero,       // {"zero":true} current position becomes 0
  CalStart,   // {"cal":"start"} begin ms-per-360 measurement
  CalStop,    // {"cal":"stop"}  finish, store yawMs360
  CalCancel,  // {"cal":"cancel"}
};

enum class Err : uint8_t {
  None,
  BadJson,     // "bad_json"
  BadType,     // "bad_type": wrong type or missing required field
  OutOfRange,  // "out_of_range"
  NoSensor,    // "no_sensor"
  Failsafe,    // "failsafe"
  LowBattery,  // "low_battery"
  BusyOta,     // "busy_ota"
  UnknownCmd,  // "unknown_cmd"
  TooLong,     // "too_long"
  TwistLimit,  // "twist_limit"
  Endstop,     // "endstop"
  Busy,        // "busy": needs motors stopped / queue full
};

// Failsafe reasons in ascending priority. The reported reason is the
// highest active one.
enum class FsReason : uint8_t { None = 0, Runtime, Timeout, Disconnect, Battery, Ota };

// Where an inbound frame came from.
// Console = the Serial console. (Not named "Serial": the Arduino core defines
// Serial as a macro on some targets.)
enum class Source : uint8_t { None, Ws, Http, Ble, Console };

const char* cmdName(Cmd c);            // "rotor", ... ; "unknown" for None
const char* errCode(Err e);            // "bad_json", ... ; "" for None
const char* errMessage(Err e);         // short English default message
const char* fsReasonName(FsReason r);  // "timeout", ... ; nullptr for None
const char* yawDirName(YawDir d);      // "left", "right", "stop"
const char* sourceName(Source s);      // "ws", "http", "ble", "serial", "none"

}  // namespace proto
