#include "protocol.h"

namespace proto {

const char* cmdName(Cmd c) {
  switch (c) {
    case Cmd::Rotor: return "rotor";
    case Cmd::Yaw: return "yaw";
    case Cmd::Stop: return "stop";
    case Cmd::Ping: return "ping";
    case Cmd::Cfg: return "cfg";
    case Cmd::Info: return "info";
    case Cmd::Log: return "log";
    case Cmd::Reboot: return "reboot";
    case Cmd::None: break;
  }
  return "unknown";
}

const char* errCode(Err e) {
  switch (e) {
    case Err::BadJson: return "bad_json";
    case Err::BadType: return "bad_type";
    case Err::OutOfRange: return "out_of_range";
    case Err::NoSensor: return "no_sensor";
    case Err::Failsafe: return "failsafe";
    case Err::LowBattery: return "low_battery";
    case Err::BusyOta: return "busy_ota";
    case Err::UnknownCmd: return "unknown_cmd";
    case Err::TooLong: return "too_long";
    case Err::TwistLimit: return "twist_limit";
    case Err::Endstop: return "endstop";
    case Err::Busy: return "busy";
    case Err::Stale: return "stale";
    case Err::None: break;
  }
  return "";
}

const char* errMessage(Err e) {
  switch (e) {
    case Err::BadJson: return "frame is not a JSON object";
    case Err::BadType: return "missing field or wrong type";
    case Err::OutOfRange: return "value out of range";
    case Err::NoSensor: return "requires the AS5600 sensor";
    case Err::Failsafe: return "failsafe active";
    case Err::LowBattery: return "battery critical, start blocked";
    case Err::BusyOta: return "update in progress";
    case Err::UnknownCmd: return "unknown command";
    case Err::TooLong: return "frame too long";
    case Err::TwistLimit: return "cable twist limit reached";
    case Err::Endstop: return "endstop reached";
    case Err::Busy: return "motors running or queue full";
    case Err::Stale: return "sent before the last stop, ignored";
    case Err::None: break;
  }
  return "";
}

const char* fsReasonName(FsReason r) {
  switch (r) {
    case FsReason::Runtime: return "runtime";
    case FsReason::Timeout: return "timeout";
    case FsReason::Disconnect: return "disconnect";
    case FsReason::Battery: return "battery";
    case FsReason::Ota: return "ota";
    case FsReason::None: break;
  }
  return nullptr;
}

const char* yawDirName(YawDir d) {
  switch (d) {
    case YawDir::Left: return "left";
    case YawDir::Right: return "right";
    case YawDir::Stop: break;
  }
  return "stop";
}

const char* sourceName(Source s) {
  switch (s) {
    case Source::Ws: return "ws";
    case Source::Http: return "http";
    case Source::Ble: return "ble";
    case Source::Console: return "serial";
    case Source::None: break;
  }
  return "none";
}

}  // namespace proto
