// Failsafe supervisor: wraps fs::FailsafeLogic, carries out the stops it
// asks for, tracks rotor run time, logs every transition. It is the master:
// every motion command goes through allowMotion() and tick() may stop the
// motors regardless of commands. Loop task only.
#pragma once

#include <stdint.h>

#include "failsafe_logic.h"
#include "protocol.h"
#include "settings_model.h"

namespace Failsafe {

void begin();
void configure(const Settings& s);  // fsTimeout, rotorMaxRun

void onFrame(const fs::ClientRef& c, uint32_t nowMs);
void onMotionCommand(const fs::ClientRef& c, uint32_t nowMs);
void onClientDisconnected(const fs::ClientRef& c, uint32_t nowMs);

// Every FS_CHECK_MS: feed FailsafeLogic (WS+BLE client count, Battery::crit,
// shared::otaActive, run limit), apply zeroSetpoints / hardStop, release the
// owner when everything is idle, publish shared::motorsIdle, log edges with
// millis() through EventLog.
void tick(uint32_t nowMs);

// Err::BusyOta / Err::LowBattery / Err::None. Reads shared::otaActive directly.
proto::Err allowMotion();

// Next rotor start command clears the Runtime reason.
void clearRuntime();

// {"t":"stop"}, Serial "s", estop flag: rotor coast now, yaw brake now, goto
// cancelled, owner released. Does NOT latch fs.active (the next command works).
// Stops every time; an "estop" event within 1 s of the previous one is not
// logged again (the panel sends the same STOP over several links).
void emergencyStop(const char* why);

bool active();
proto::FsReason reason();
uint32_t rotorRunS();  // continuous rotor run time, s

}  // namespace Failsafe
