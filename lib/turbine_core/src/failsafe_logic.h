// Failsafe state machine, hardware independent. failsafe.cpp feeds it and
// carries out the stops it asks for.
//
// Rules (see docs/PROTOCOL.md, "Failsafe"):
//  - The client that issued the last accepted motion command is the motion
//    OWNER. Only the owner's silence counts: if the owner sends nothing for
//    timeoutMs while it owns motion, trip Timeout. Another phone pinging the
//    panel does not keep someone else's motion alive.
//  - The owner disconnecting trips Disconnect at once. The WS+BLE client
//    count dropping from >0 to 0 also trips Disconnect (an edge, not a state).
//  - Timeout / Disconnect / Runtime trips ZERO THE SETPOINTS (rotor target 0
//    with the fixed failsafe ramp, yaw stop, goto cancelled) and release the
//    owner. Nothing restarts by itself after recovery.
//  - Timeout and Disconnect clear on the next valid frame from any client.
//    Runtime clears via clearRuntime() (next rotor start command).
//  - Battery mirrors the latched batteryCrit input, Ota mirrors otaActive.
//    Both request a hard stop and block motion (allowMotion()).
//  - The owner is released when all motion has stopped (releaseOwner()).
#pragma once

#include <stdint.h>

#include "protocol.h"

namespace fs {

struct ClientRef {
  proto::Source src = proto::Source::None;
  uint32_t id = 0;  // WS client id, BLE conn handle, HTTP remote IPv4, 0 for Serial
  bool valid() const { return src != proto::Source::None; }
  bool operator==(const ClientRef& o) const { return src == o.src && id == o.id; }
  bool operator!=(const ClientRef& o) const { return !(*this == o); }
};

struct Inputs {
  uint32_t nowMs = 0;
  uint16_t clients = 0;     // WS + BLE connected clients
  bool batteryCrit = false; // latched by the battery module (with hysteresis)
  bool otaActive = false;
  bool runLimitHit = false; // rotor ran longer than rotorMaxRun
};

struct Result {
  bool tripped = false;                               // a reason became active in this update
  proto::FsReason trippedReason = proto::FsReason::None;  // highest one that tripped now
  bool zeroSetpoints = false;  // ramp rotor to 0 (FS_RAMP_MS), stop yaw, cancel goto
  bool hardStop = false;       // rotor coast now, yaw brake now (Battery / Ota)
};

class FailsafeLogic {
 public:
  void setTimeout(uint32_t ms) { timeoutMs_ = ms; }
  uint32_t timeout() const { return timeoutMs_; }

  // Any valid frame from c. Refreshes the owner's last-seen time if c is the
  // owner and clears Timeout and Disconnect.
  void onFrame(const ClientRef& c, uint32_t nowMs);
  // An accepted motion command from c: c becomes the owner.
  void onMotionCommand(const ClientRef& c, uint32_t nowMs);
  // All motors idle and no goto: no supervision needed.
  void releaseOwner();
  // Transport says c went away. Trips Disconnect if c is the owner (applied on
  // the next update()).
  void onClientDisconnected(const ClientRef& c, uint32_t nowMs);

  Result update(const Inputs& in);

  void clearRuntime();

  bool active() const { return mask_ != 0; }
  proto::FsReason reason() const;  // highest active, None if inactive
  uint8_t mask() const { return mask_; }  // bit (1 << reason)
  bool has(proto::FsReason r) const { return (mask_ >> (uint8_t)r) & 1u; }
  // Err::BusyOta while Ota, Err::LowBattery while Battery, else Err::None.
  proto::Err allowMotion() const;

  bool hasOwner() const { return owner_.valid(); }
  ClientRef owner() const { return owner_; }

 private:
  void set(proto::FsReason r) { mask_ |= (uint8_t)(1u << (uint8_t)r); }
  void clear(proto::FsReason r) { mask_ &= (uint8_t)~(1u << (uint8_t)r); }

  uint32_t timeoutMs_ = 500;
  uint8_t mask_ = 0;
  ClientRef owner_;
  uint32_t ownerSeenMs_ = 0;
  bool ownerLost_ = false;   // pending Disconnect from onClientDisconnected
  uint16_t lastClients_ = 0;
};

}  // namespace fs
