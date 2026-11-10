// Send-order guard for HTTP commands.
//
// Every POST /api/cmd is a separate TCP connection (the server answers with
// "Connection: close"), so on a lossy link a request can reach the ESP after
// a later one. The panel stamps rotor / yaw / stop frames sent over HTTP with
// an increasing "seq" (its millisecond clock). A motion frame that is older
// than the last stop of the same axis from the same client must not run:
// it would restart a motor the user has just stopped.
//
// Axes: {"t":"stop"} stops both. Rotor: speed 0 (with or without brake) is a
// stop, speed > 0 is motion. Yaw: dir "stop" is a stop, dir left / right and
// goto are motion. Everything else (hold, zero, cal, cfg, ping, ...) and any
// frame without "seq" passes unchecked: a hold can never restart the yaw.
//
// A stop is remembered for kWindowMs, so a client whose clock jumped back (or
// a phone that got the IP of a phone that left) is refused for a bounded time
// at most. Pure logic; the firmware uses one instance from the HTTP handler
// task only.
#pragma once

#include <stdint.h>

#include "cmd_parser.h"

class StopSeqGuard {
 public:
  static constexpr uint8_t kSlots = 4;          // clients remembered, least recently used replaced
  static constexpr uint32_t kWindowMs = 60000;  // how long a stop blocks older motion frames

  // client: HTTP remote IPv4. Records stops; returns false for a stale
  // motion frame (the caller must not execute it).
  bool accept(uint32_t client, const Command& c, uint32_t nowMs);

  void reset();

 private:
  struct Axis {
    bool has = false;
    uint32_t seq = 0;   // newest stop seen
    uint32_t atMs = 0;  // when it was seen (local clock)
  };
  struct Slot {
    bool used = false;
    uint32_t client = 0;
    uint32_t touchMs = 0;  // for LRU replacement
    Axis rotor, yaw;
  };

  Slot* find(uint32_t client);
  Slot* take(uint32_t client, uint32_t nowMs);

  Slot slots_[kSlots];
};
