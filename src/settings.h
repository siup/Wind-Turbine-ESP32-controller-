// Persistent runtime settings (NVS namespace "cfg"), driven by the
// descriptor table in settings_model.h. Loop task only.
#pragma once

#include <stdint.h>

#include "settings_model.h"

namespace SettingsStore {

// Load every field from NVS; missing or invalid stored values fall back to
// the default for that field (logged). Keeps a boot snapshot to tell which
// kFieldReboot fields changed since boot.
void begin();

const Settings& get();

// Increments on every commit / factory reset (sent as "cfgRev" in state so
// HTTP pollers can refetch cfg).
uint32_t revision();

// Replace the settings (already validated). Bumps the revision and schedules
// a save of the changed keys after SETTINGS_SAVE_DELAY_MS.
void commit(const Settings& next);

// Defaults for every field, saved at once. Does not touch the yaw position
// namespace (balance / AS5600 offset survive).
void factoryReset();

// True if a kFieldReboot field differs from the value the device booted with.
bool rebootPending();

// Deferred save.
void tick(uint32_t nowMs);

// Write pending changes now (before a reboot).
void flush();

}  // namespace SettingsStore
