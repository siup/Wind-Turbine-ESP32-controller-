// Failsafe supervisor on top of fs::FailsafeLogic (see failsafe_logic.h and
// docs/PROTOCOL.md, "Failsafe"). This file only feeds the inputs, carries out
// the stops the logic asks for, keeps the rotor run timer and logs edges.

#include "failsafe.h"

#include <Arduino.h>

#include "battery.h"
#include "config.h"
#include "event_log.h"
#include "log.h"
#include "motors.h"
#include "shared_state.h"
#include "transport_ble.h"
#include "transport_wifi.h"
#include "yaw.h"

namespace {

fs::FailsafeLogic g_logic;
uint16_t g_maxRunS = ROTOR_MAX_RUN_S;
bool g_running = false;      // rotor output > 0 since g_runStartMs
uint32_t g_runStartMs = 0;
uint32_t g_lastCheckMs = 0;
bool g_checkNow = true;      // run the next tick() without waiting FS_CHECK_MS
uint8_t g_lastMask = 0;      // mask already logged
uint32_t g_ownerSeenMs = 0;  // last frame from the motion owner (for the log text)
uint16_t g_lastClients = 0;

constexpr uint8_t bitOf(proto::FsReason r) { return (uint8_t)(1u << (uint8_t)r); }

// Highest priority first.
constexpr proto::FsReason kOrder[] = {proto::FsReason::Ota, proto::FsReason::Battery,
                                      proto::FsReason::Disconnect, proto::FsReason::Timeout,
                                      proto::FsReason::Runtime};

proto::FsReason highest(uint8_t mask) {
  for (proto::FsReason r : kOrder) {
    if (mask & bitOf(r)) return r;
  }
  return proto::FsReason::None;
}

void formatClient(const fs::ClientRef& c, char* out, size_t cap) {
  if (c.src == proto::Source::Http) {
    // HTTP clients are keyed by their IPv4 (first octet in the low byte).
    snprintf(out, cap, "http@%u.%u.%u.%u", (unsigned)(c.id & 0xff), (unsigned)((c.id >> 8) & 0xff),
             (unsigned)((c.id >> 16) & 0xff), (unsigned)((c.id >> 24) & 0xff));
  } else if (c.src == proto::Source::Ws || c.src == proto::Source::Ble) {
    snprintf(out, cap, "%s#%lu", proto::sourceName(c.src), (unsigned long)c.id);
  } else {
    snprintf(out, cap, "%s", proto::sourceName(c.src));  // "serial" / "none"
  }
}

uint32_t runSeconds(uint32_t nowMs) { return g_running ? (nowMs - g_runStartMs) / 1000u : 0; }

void logTrip(proto::FsReason r, uint32_t nowMs, const fs::ClientRef& owner) {
  char who[24];
  formatClient(owner, who, sizeof(who));
  switch (r) {
    case proto::FsReason::Timeout:
      EventLog::add('w', "fs_timeout", "owner %s silent %lu ms", who,
                    (unsigned long)(nowMs - g_ownerSeenMs));
      break;
    case proto::FsReason::Disconnect:
      if (owner.valid()) {
        EventLog::add('w', "fs_disconnect", "owner %s, clients %u", who, (unsigned)g_lastClients);
      } else {
        EventLog::add('w', "fs_disconnect", "last client left");
      }
      break;
    case proto::FsReason::Battery:
      EventLog::add('w', "fs_battery", "%lu mV, motors stopped", (unsigned long)Battery::mv());
      break;
    case proto::FsReason::Ota:
      EventLog::add('w', "fs_ota", "update running, motors locked");
      break;
    case proto::FsReason::Runtime:
      EventLog::add('w', "fs_runtime", "rotor ran %lu s, owner %s", (unsigned long)runSeconds(nowMs), who);
      break;
    default:
      break;
  }
}

// Log reasons that became active since the last call, and the full clear.
void logEdges(uint32_t nowMs, const fs::ClientRef& owner) {
  const uint8_t m = g_logic.mask();
  const uint8_t added = (uint8_t)(m & ~g_lastMask);
  for (proto::FsReason r : kOrder) {
    if (added & bitOf(r)) logTrip(r, nowMs, owner);
  }
  if (m == 0 && g_lastMask != 0) {
    const char* was = proto::fsReasonName(highest(g_lastMask));
    EventLog::add('i', "fs_clear", "was %s", was ? was : "?");
  }
  g_lastMask = m;
}

bool allIdle() { return rotorMotor.idle() && yaw.idle(); }

}  // namespace

namespace Failsafe {

void begin() {
  g_logic.setTimeout(FS_TIMEOUT_MS);
  g_maxRunS = ROTOR_MAX_RUN_S;
  g_running = false;
  g_checkNow = true;
  g_lastMask = g_logic.mask();
  shared::motorsIdle.store(allIdle());
}

void configure(const Settings& s) {
  g_logic.setTimeout(s.fsTimeout);
  g_maxRunS = s.rotorMaxRun;
}

void onFrame(const fs::ClientRef& c, uint32_t nowMs) {
  const fs::ClientRef owner = g_logic.owner();
  if (g_logic.hasOwner() && c == owner) g_ownerSeenMs = nowMs;
  g_logic.onFrame(c, nowMs);
  logEdges(nowMs, owner);
}

void onMotionCommand(const fs::ClientRef& c, uint32_t nowMs) {
  g_logic.onMotionCommand(c, nowMs);
  g_ownerSeenMs = nowMs;
  logEdges(nowMs, c);
}

void onClientDisconnected(const fs::ClientRef& c, uint32_t nowMs) {
  g_logic.onClientDisconnected(c, nowMs);
  g_checkNow = true;  // act on the next loop pass, not up to FS_CHECK_MS later
}

void tick(uint32_t nowMs) {
  // Cheap, every call: the OTA handler reads this before accepting an upload.
  shared::motorsIdle.store(allIdle());

  if (!g_checkNow && (int32_t)(nowMs - g_lastCheckMs) < (int32_t)FS_CHECK_MS) return;
  g_checkNow = false;
  g_lastCheckMs = nowMs;

  // Continuous rotor run time.
  if (rotorMotor.actual() > 0) {
    if (!g_running) {
      g_running = true;
      g_runStartMs = nowMs;
    }
  } else {
    g_running = false;
  }

  fs::Inputs in;
  in.nowMs = nowMs;
  g_lastClients = (uint16_t)(TransportWifi::wsClients() + TransportBle::clientCount());
  in.clients = g_lastClients;
  in.batteryCrit = Battery::crit();
  in.otaActive = shared::otaActive.load();
  in.runLimitHit = g_maxRunS > 0 && runSeconds(nowMs) >= g_maxRunS;

  const fs::ClientRef ownerBefore = g_logic.owner();
  const fs::Result r = g_logic.update(in);

  // FailsafeLogic repeats zeroSetpoints (Runtime is level triggered) and
  // hardStop (while Battery / Ota last) on every update. The unconditional
  // stop happens on the edge; afterwards only what still moves is stopped,
  // so an idle bridge is not woken up (and re-braked) every 50 ms.
  const uint8_t hardBits = bitOf(proto::FsReason::Battery) | bitOf(proto::FsReason::Ota);
  const bool hardEdge = (g_logic.mask() & hardBits & (uint8_t)~g_lastMask) != 0;

  if (r.zeroSetpoints) {
    // An idle rotor (maybe braking) has no setpoint to zero.
    if (!rotorMotor.idle()) rotorMotor.setSpeedWithRamp(0, FS_RAMP_MS);
    if (!yaw.idle()) yaw.stop(true);
  }
  if (hardEdge) {
    rotorMotor.stop();  // coast now (also releases a rotor brake)
    yaw.stop(true);
  } else if (r.hardStop || in.otaActive) {
    if (!rotorMotor.idle()) rotorMotor.stop();
    if (!yaw.idle()) yaw.stop(true);
  }

  logEdges(nowMs, ownerBefore);

  if (allIdle()) g_logic.releaseOwner();
  shared::motorsIdle.store(allIdle());
}

proto::Err allowMotion() {
  // Direct read: an upload may have started after the last tick.
  if (shared::otaActive.load()) return proto::Err::BusyOta;
  const proto::Err e = g_logic.allowMotion();
  if (e != proto::Err::None) return e;
  // Battery latch may have changed since the last tick as well.
  if (Battery::crit()) return proto::Err::LowBattery;
  return proto::Err::None;
}

void clearRuntime() {
  if (g_logic.has(proto::FsReason::Runtime)) {
    // Restart the run timer, else a restart during the ramp-down would trip
    // again at once.
    g_running = false;
  }
  g_logic.clearRuntime();
  logEdges(millis(), g_logic.owner());
}

void emergencyStop(const char* why) {
  rotorMotor.stop();
  yaw.stop(true);
  g_logic.releaseOwner();
  shared::motorsIdle.store(allIdle());
  EventLog::add('i', "estop", "%s", (why && why[0]) ? why : "stop");
}

bool active() { return g_logic.active(); }

proto::FsReason reason() { return g_logic.reason(); }

uint32_t rotorRunS() { return runSeconds(millis()); }

}  // namespace Failsafe
