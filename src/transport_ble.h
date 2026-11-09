// BLE transport (NimBLE-Arduino 2.x): one service BLE_SERVICE_UUID with
//   cmd   (write + write without response): one JSON frame per write
//   state (read + notify): short state frames, errors, pongs
// Name = appDeviceName(), in the scan response (128-bit UUID + flags fill
// the 31-byte advertising packet). Preferred MTU BLE_PREFERRED_MTU,
// connection interval 15-30 ms requested on connect.
//
// Advertising policy (bleMode): off; auto = only while no WiFi station is
// associated (radio coexistence hurts WiFi latency and battery); always.
// A connected BLE client is never kicked because WiFi came up.
//
// Without ENABLE_BLE every function is a cheap no-op.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "settings_model.h"

namespace TransportBle {

void begin(const Settings& s);
void configure(const Settings& s);  // bleMode
void tick(uint32_t nowMs, uint8_t wifiStations);  // advertising policy
// Notify all connected peers. Frames longer than (peer MTU - 3) are skipped
// for that peer (logged once per connection).
void notify(const char* data, size_t len);
uint8_t clientCount();
bool rssi(int8_t& out);

}  // namespace TransportBle
