// The few flags shared across tasks. Everything else is owned by the
// Arduino loop task and must only be touched from there.
//
// Task map:
//   loopTask (core 1)   all control state: motors, yaw, battery, failsafe,
//                       settings, telemetry, event log
//   async_tcp (core 0)  WS / HTTP callbacks: only CmdQueue::push*(),
//                       TransportWifi cache reads, the atomics below
//   NimBLE host         GATT callbacks: only CmdQueue::push*() and atomics
#pragma once

#include <atomic>

namespace shared {

// Set by the OTA upload handler (async_tcp) when an upload starts. Read by
// Failsafe::allowMotion() directly so a command already queued before the
// upload cannot start a motor.
extern std::atomic<bool> otaActive;

// Published by the loop every tick: true when both motors are idle and no
// goto is running. Read by the OTA handler before accepting an upload.
extern std::atomic<bool> motorsIdle;

// Raised by CmdQueue when a {"t":"stop"} frame arrives (even if the queue is
// full). The loop checks it first thing every iteration.
extern std::atomic<bool> estopRequested;

}  // namespace shared
