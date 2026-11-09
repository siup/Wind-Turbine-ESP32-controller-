// Glue implemented in main.cpp so that modules (mostly commands.cpp) can ask
// the application for cross-module actions without including each other.
// Loop task only.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "settings_model.h"

// Push settings to every module (motors, yaw, battery, failsafe, wifi tx
// power, BLE mode, LED). `prev` lets modules react to specific changes
// (e.g. yawInvert flip zeroes the yaw balance). Called once at boot with
// prev == next.
void appApplySettings(const Settings& prev, const Settings& next);

// Stop motors, persist yaw balance and pending settings, restart after delayMs.
void appRequestReboot(uint32_t delayMs, const char* why);

// Send a frame to every WebSocket client; also notify BLE peers if toBle.
void appBroadcast(const char* data, size_t len, bool toBle);

// "Turbina-AB12" (from the MAC) or the `name` setting if set. Valid after setup() started.
const char* appDeviceName();

// esp_reset_reason() as a short string ("poweron", "task_wdt", "brownout", ...).
const char* appResetReason();
