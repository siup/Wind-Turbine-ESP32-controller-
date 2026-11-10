// WiFi access point, HTTP server, WebSocket, static panel from LittleFS.
//
//   SSID appDeviceName(), password from settings, channel apChan,
//   IP 192.168.4.1, max AP_MAX_CLIENTS stations, TX power from settings.
//   GET  /            panel from LittleFS (index.html.gz served with
//                     Content-Encoding: gzip). If LittleFS or index is
//                     missing: a small built-in page with an upload form
//                     for littlefs.bin (POST /update?type=fs).
//   GET  /api/state   last telemetry frame (cache)
//   POST /api/cmd     body = one JSON frame. Pre-validated in the handler
//                     with the pure parser, then queued. Immediate reply:
//                     err (parse error / queue full / stale), pong (ping),
//                     cfg / info / log from the caches, otherwise ack. Never
//                     blocks the async_tcp task waiting for the loop. A motion
//                     frame whose "seq" is older than the client's last stop
//                     is refused with err stale (StopSeqGuard).
//   WS   /ws          frames both ways (text). On connect: setNoDelay(true)
//                     on the TCP socket, push MsgKind::Connected.
//   Captive portal DNS when ENABLE_CAPTIVE and settings.captive.
//
// Without ENABLE_WIFI every function is a cheap no-op.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "settings_model.h"

namespace TransportWifi {

// Also mounts LittleFS, calls Ota::attach(server) and gives the OTA handler
// the password the access point actually runs with (it changes only with a
// restart, like the AP).
void begin(const Settings& s);

// cleanupClients() once a second, close WS clients silent for
// WS_SILENT_CLOSE_MS, refresh station count and RSSI
// (esp_wifi_ap_get_sta_list) once a second.
void tick(uint32_t nowMs);

void sendTo(uint32_t clientId, const char* data, size_t len);
// All WS clients. Clients whose send queue is full are skipped for this
// frame instead of being disconnected.
void broadcast(const char* data, size_t len);

uint8_t wsClients();
// WS client still connected (its slot is removed before its Disconnected
// event is queued). Any task.
bool hasClient(uint32_t clientId);
uint8_t stations();
bool rssi(int8_t& out);  // weakest associated station, false if none

void setTxPower(uint8_t qdbm);  // snapped, applied live

enum class Cache : uint8_t { State, Cfg, Info, Log };
// Loop task: store a copy for HTTP handlers (one buffer per cache behind a
// mutex; a publish that cannot get the mutex within 5 ms is skipped).
void publish(Cache which, const char* data, size_t len);

}  // namespace TransportWifi
