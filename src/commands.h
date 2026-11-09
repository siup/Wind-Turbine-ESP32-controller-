// Command layer: the only place in src/ that touches JSON (through the pure
// parser and frame builders in lib/turbine_core). Loop task only.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "cmd_queue.h"

// Where replies for one inbound frame go.
class ReplySink {
 public:
  virtual ~ReplySink() = default;
  virtual void reply(const char* data, size_t len) = 0;
  // false for BLE (acks only when the frame carried an "id") and for HTTP
  // (the HTTP handler already answered).
  virtual bool wantsAck(bool hasId) const { (void)hasId; return true; }
  virtual bool wantsReplies() const { return true; }  // false: execute silently
};

namespace Commands {

void begin();

// Parse + execute one Command message (see docs/PROTOCOL.md for semantics):
//  - every valid frame -> Failsafe::onFrame(client)
//  - OTA active -> everything except stop / ping / info / log -> err busy_ota
//  - rotor speed > 0: Failsafe::allowMotion(), clearRuntime(), clamp to rotorMax,
//    rotorMotor.setSpeed(), Failsafe::onMotionCommand()
//  - rotor speed 0: ramp to 0, or brake() if "brake":true
//  - yaw dir left/right: allowMotion(), yaw.move(), onMotionCommand(); dir stop: yaw.stop(true)
//  - yaw hold: yaw.hold(); goto: allowMotion(), yaw.gotoAngle(), onMotionCommand()
//  - yaw zero / cal: yaw.zero() / yaw.calibrate(); cal stop commits yawMs360
//  - stop: Failsafe::emergencyStop()
//  - cfg get -> cfg frame; cfg set -> re-parse against current settings,
//    kFieldIdleOnly fields need idle motors (err busy), commit, appApplySettings(),
//    ack, then broadcast the new cfg frame to all WS clients; cfg reset likewise
//  - info / log -> frames; reboot -> ack, appRequestReboot(300)
void handle(const InMsg& msg, ReplySink& sink);

// info + cfg + state to a freshly connected client.
void sendHello(ReplySink& sink);

// Snapshot of all modules -> state frame. signatureOut (optional) receives
// stateSignature() of the snapshot.
size_t buildTelemetry(char* out, size_t cap, bool brief, uint32_t* signatureOut);

size_t buildCfgFrame(char* out, size_t cap);
size_t buildInfoFrame(char* out, size_t cap);
size_t buildLogFrame(char* out, size_t cap);

}  // namespace Commands
