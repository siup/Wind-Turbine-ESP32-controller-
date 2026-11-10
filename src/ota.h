// Built-in OTA (replaces ElegantOTA, see README): POST /update?type=fw|fs,
// multipart file upload, HTTP Basic Auth OTA_USER / AP password.
//   - refused with 409 unless shared::motorsIdle
//   - sets shared::otaActive before the first byte is written: motors stop,
//     commands get err busy_ota, fs.reason = "ota"
//   - type=fs: LittleFS.end() first, Update.begin(size, U_FLASHFS) (the
//     "spiffs" subtype partition of min_spiffs.csv holds LittleFS)
//   - wrong password: 401; the Basic challenge only goes to requests that
//     carried no credentials (XHR from the panel gets JSON, no browser popup)
//   - responds 200 {"ok":true} or 500 {"ok":false,"error":"..."}
//   - the device restarts ~1 s after the upload ends, success or not
//     (spec: OTA lock is released by a restart)
// GET /update redirects to /#ota (the panel has the upload UI).
// Without ENABLE_OTA: attach() registers nothing.
#pragma once

#include <stdint.h>

class AsyncWebServer;

namespace Ota {
void attach(AsyncWebServer& server);
// Loop task (appApplySettings): copy of the AP password used for Basic Auth,
// kept under a lock so the async_tcp handler never reads Settings directly.
void setPassword(const char* apPass);
void tick(uint32_t nowMs);  // loop: logs progress / result, schedules the restart
bool active();
uint8_t progressPct();
}  // namespace Ota
