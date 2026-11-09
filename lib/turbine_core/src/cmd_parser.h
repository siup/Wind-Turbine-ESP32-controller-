// JSON command parser (protocol v1). Pure: no Arduino, no globals, reentrant,
// so transports may call peekCommandType() / parseCommand() from their own
// tasks for pre-validation.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"
#include "settings_model.h"

struct Command {
  proto::Cmd type = proto::Cmd::None;
  bool hasId = false;  // optional "id" (uint32) echoed in ack / err / pong
  uint32_t id = 0;

  // rotor: {"t":"rotor","speed":0..100[,"brake":true]}
  uint8_t speed = 0;
  bool brake = false;

  // yaw
  proto::YawAction yawAction = proto::YawAction::Move;
  proto::YawDir dir = proto::YawDir::Stop;  // Move / Hold
  uint8_t yawSpeed = 0;                     // Move: 20..100, 0 = use settings
  float gotoDeg = 0.0f;                     // Goto: [0, 360)

  // cfg: {"t":"cfg","get":true} | {"t":"cfg","reset":true} | {"t":"cfg",<key>:<value>,...}
  bool cfgGet = false;    // also true for a bare {"t":"cfg"}
  bool cfgReset = false;
  uint64_t cfgMask = 0;   // bit i set = kSettingsFields[i] present in a cfg-set
};

struct ParseResult {
  proto::Err err = proto::Err::None;
  const char* field = nullptr;  // offending JSON key (static string) or nullptr
};

// Cheap scan for the "t" value without building a document. Used by
// transports to put {"t":"stop"} at the front of the queue. Returns
// Cmd::None if it cannot tell.
proto::Cmd peekCommandType(const char* json, size_t len);

// Full parse and validation.
//  - len > kMaxCmdLen            -> TooLong
//  - not a JSON object           -> BadJson
//  - "t" missing / not a string  -> BadType, field "t"
//  - unknown "t"                 -> UnknownCmd
//  - wrong type / missing field  -> BadType, field = key
//  - value outside range / enum  -> OutOfRange, field = key
//  - unknown keys are ignored
// rotor.speed accepts integers or floats (rounded); yaw "speed" 20..100.
// A yaw frame must carry exactly one of dir / hold / goto / zero / cal.
//
// cfg-set: if cfgBase and cfgOut are given, *cfgOut = *cfgBase, then the
// battery preset is applied first if "batType" is 0 or 1, then every known
// key is applied and range-checked, then settingsValidate() runs (cross-field
// errors report the field settingsValidate() returns). Nothing is written to
// *cfgOut unless the whole patch is valid. Secret keys are writable.
// If cfgBase is nullptr, cfg keys are only type/range checked.
ParseResult parseCommand(const char* json, size_t len, Command& out,
                         const Settings* cfgBase = nullptr, Settings* cfgOut = nullptr);
