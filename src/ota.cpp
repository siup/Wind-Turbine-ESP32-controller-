// Built-in OTA upload: POST /update?type=fw|fs (multipart), Basic Auth.
//
// Threading: the upload and completion handlers run in the async_tcp task,
// Ota::tick() runs in the loop task. They share only atomics plus a short
// result string guarded by a portMUX. The Update singleton is touched from
// async_tcp only.
//
// Flow of one upload request:
//   first file chunk  -> auth, type, "already running", motors idle checks.
//                        Accepted: otaActive = true, Update.begin().
//                        Refused: the verdict is stored in the request and
//                        the rest of the body is drained and ignored.
//   further chunks    -> Update.write()
//   last chunk        -> Update.end(true)
//   completion        -> 200 {"ok":true} / 4xx / 500 {"ok":false,"error":".."}
//   client vanished   -> Update.abort()
// Ota::tick() logs the start and the result and asks for a restart ~1 s
// after any accepted upload ended (success, failure or abort).
#include "ota.h"

#include "config.h"

#if ENABLE_OTA

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Update.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>

#include "app.h"
#include "event_log.h"
#include "log.h"
#include "shared_state.h"

namespace {

constexpr const char* kRealm = "Turbina";
constexpr uint32_t kRebootAfterMs = 1000;  // after the upload ended
constexpr uint32_t kStallMs = 15000;       // no data for this long: give up (reboot)
constexpr uint8_t kImageMagic = 0xE9;      // first byte of an ESP32 app image

enum Phase : uint8_t { kIdle = 0, kRunning, kEnded };

// --- shared between async_tcp and loop -------------------------------------
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
char s_pass[64] = AP_PASSWORD;  // Basic Auth password (copy, see setPassword)
char s_result[48] = "";         // result text, written before s_phase = kEnded

std::atomic<uint8_t> s_phase{kIdle};
std::atomic<bool> s_ok{false};
std::atomic<bool> s_isFs{false};
std::atomic<uint8_t> s_progress{0};
std::atomic<uint32_t> s_written{0};
std::atomic<uint32_t> s_lastDataMs{0};

// --- async_tcp only ---------------------------------------------------------
AsyncWebServerRequest* s_owner = nullptr;  // request that owns Update

// --- loop only --------------------------------------------------------------
bool s_loggedStart = false;
bool s_loggedEnd = false;
bool s_rebootAsked = false;
uint32_t s_endedAtMs = 0;
uint8_t s_loggedPct = 0;

// Per-request verdict, kept in request->_tempObject (the request frees it).
struct UploadCtx {
  uint16_t status;  // 200 accepted, else the HTTP status to answer with
  bool fileDone;    // the first file part ended; later parts are ignored
  char err[32];
};

void copyText(char* dst, size_t cap, const char* src) {
  // JSON-safe copy: the result ends up inside a JSON string.
  size_t i = 0;
  for (; src && src[i] && i + 1 < cap; i++) {
    const char c = src[i];
    dst[i] = (c == '"' || c == '\\' || (unsigned char)c < 0x20) ? '\'' : c;
  }
  dst[i] = '\0';
}

void finish(bool ok, const char* msg) {
  portENTER_CRITICAL(&s_mux);
  copyText(s_result, sizeof(s_result), msg);
  portEXIT_CRITICAL(&s_mux);
  s_ok.store(ok);
  if (ok) s_progress.store(100);
  s_phase.store(kEnded);
}

bool authOk(AsyncWebServerRequest* req) {
  char pass[sizeof(s_pass)];
  portENTER_CRITICAL(&s_mux);
  memcpy(pass, s_pass, sizeof(pass));
  portEXIT_CRITICAL(&s_mux);
  return req->authenticate(OTA_USER, pass);
}

void sendResult(AsyncWebServerRequest* req, int code, bool ok, const char* err) {
  char body[96];
  if (ok) {
    snprintf(body, sizeof(body), "{\"ok\":true}");
  } else {
    char e[48];
    copyText(e, sizeof(e), err ? err : "error");
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", e);
  }
  AsyncWebServerResponse* r = req->beginResponse(code, "application/json", body);
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}

// Wrong or missing credentials. The panel and the fallback page send their
// own Authorization header from XHR; answering those with a Basic challenge
// would make the browser pop up its own login dialog on top of the panel's
// message, so the challenge only goes to requests without credentials.
void sendUnauthorized(AsyncWebServerRequest* req) {
  if (req->hasHeader("Authorization")) {
    sendResult(req, 401, false, "bad password");
  } else {
    req->requestAuthentication(AsyncAuthType::AUTH_BASIC, kRealm);
  }
}

void refuse(UploadCtx* ctx, uint16_t status, const char* err) {
  ctx->status = status;
  copyText(ctx->err, sizeof(ctx->err), err);
}

// The owner's connection closed. Abort a half-written image.
void onOwnerGone(AsyncWebServerRequest* req) {
  if (s_owner != req) return;
  s_owner = nullptr;
  if (s_phase.load() == kRunning) {
    Update.abort();
    finish(false, "aborted: client disconnected");
  }
}

// First chunk of the first file part: decide and, if accepted, start Update.
void start(AsyncWebServerRequest* req, UploadCtx* ctx, const uint8_t* data, size_t len) {
  if (!authOk(req)) {
    ctx->status = 401;
    return;
  }

  bool fs = false;
  const AsyncWebParameter* p = req->getParam("type");
  if (p) {
    if (p->value() == "fs") {
      fs = true;
    } else if (p->value() != "fw") {
      refuse(ctx, 400, "bad type, use fw or fs");
      return;
    }
  }

  if (s_owner != nullptr || s_phase.load() != kIdle) {
    refuse(ctx, 409, "busy");
    return;
  }
  // Decided before otaActive is raised: a refused upload must not trip the
  // failsafe. A motion command racing with this check is stopped by the
  // failsafe as soon as otaActive is set.
  if (!shared::motorsIdle.load()) {
    refuse(ctx, 409, "motors running");
    return;
  }
  if (fs && len > 0 && data[0] == kImageMagic) {
    refuse(ctx, 400, "firmware image sent as fs");
    return;
  }

  // Accepted. From here on the upload always ends in a restart.
  shared::otaActive.store(true);
  s_owner = req;
  s_isFs.store(fs);
  s_written.store(0);
  s_progress.store(0);
  s_lastDataMs.store(millis());
  s_phase.store(kRunning);
  ctx->status = 200;
  req->onDisconnect([req]() { onOwnerGone(req); });

  if (fs) LittleFS.end();  // the image overwrites the mounted partition
  // U_FLASHFS picks the "spiffs" subtype partition first (LittleFS lives
  // there with min_spiffs.csv) and falls back to fat / littlefs subtypes.
  if (!Update.begin(UPDATE_SIZE_UNKNOWN, fs ? U_FLASHFS : U_FLASH)) {
    const char* e = Update.errorString();
    refuse(ctx, 500, e);
    finish(false, e);
  }
}

void onUpload(AsyncWebServerRequest* req, const String& filename, size_t index, uint8_t* data, size_t len,
              bool final) {
  (void)filename;
  UploadCtx* ctx = static_cast<UploadCtx*>(req->_tempObject);
  if (!ctx) {
    if (index != 0) return;
    ctx = static_cast<UploadCtx*>(calloc(1, sizeof(UploadCtx)));
    if (!ctx) return;  // completion answers 500
    req->_tempObject = ctx;
    start(req, ctx, data, len);
  } else if (index == 0 && ctx->fileDone) {
    return;  // a second file part: only the first one counts
  }

  if (ctx->fileDone || ctx->status != 200 || s_owner != req || s_phase.load() != kRunning) return;

  if (len > 0) {
    if (Update.write(data, len) != len) {
      const char* e = Update.errorString();
      Update.abort();
      refuse(ctx, 500, e);
      finish(false, e);
      ctx->fileDone = true;
      return;
    }
    const uint32_t written = s_written.load() + len;
    s_written.store(written);
    s_lastDataMs.store(millis());
    const size_t total = req->contentLength();
    if (total > 0) {
      uint32_t pct = (uint32_t)(((uint64_t)(index + len) * 100u) / total);
      if (pct > 99) pct = 99;  // 100 only after Update.end() succeeded
      s_progress.store((uint8_t)pct);
    }
  }

  if (final) {
    ctx->fileDone = true;
    if (s_written.load() == 0) {
      Update.abort();
      refuse(ctx, 400, "empty file");
      finish(false, "empty file");
    } else if (Update.end(true)) {
      finish(true, "ok");
    } else {
      const char* e = Update.errorString();
      refuse(ctx, 500, e);
      finish(false, e);
    }
  }
}

void onRequestDone(AsyncWebServerRequest* req) {
  UploadCtx* ctx = static_cast<UploadCtx*>(req->_tempObject);
  if (!ctx) {
    // No file part arrived (or out of memory before the first chunk).
    if (!authOk(req)) {
      sendUnauthorized(req);
    } else {
      sendResult(req, 400, false, "no file in request");
    }
    return;
  }

  if (ctx->status == 401) {
    sendUnauthorized(req);
    return;
  }

  if (ctx->status == 200 && s_owner == req && s_phase.load() == kRunning) {
    // Body ended without the closing multipart boundary.
    Update.abort();
    refuse(ctx, 500, "incomplete upload");
    finish(false, "incomplete upload");
  }

  if (ctx->status == 200) {
    sendResult(req, 200, true, nullptr);
  } else {
    sendResult(req, ctx->status, false, ctx->err);
  }
}

}  // namespace

namespace Ota {

void attach(AsyncWebServer& server) {
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest* req) { req->redirect("/#ota"); });
  server.on("/update", HTTP_POST, onRequestDone, onUpload);
  LOGI("ota: POST /update?type=fw|fs ready");
}

void setPassword(const char* apPass) {
  // Same fallback as the access point: an invalid stored password means the
  // AP runs with AP_PASSWORD, so OTA accepts that one too.
  const char* p = (apPass && strlen(apPass) >= 8) ? apPass : AP_PASSWORD;
  portENTER_CRITICAL(&s_mux);
  strlcpy(s_pass, p, sizeof(s_pass));
  portEXIT_CRITICAL(&s_mux);
}

void tick(uint32_t nowMs) {
  const uint8_t phase = s_phase.load();
  if (phase == kIdle) return;

  const char* kind = s_isFs.load() ? "fs" : "fw";
  if (!s_loggedStart) {
    s_loggedStart = true;
    EventLog::add('w', "ota_start", "%s upload started, motors locked", kind);
  }

  if (phase == kRunning && !s_loggedEnd) {
    const uint8_t pct = s_progress.load();
    if (pct >= s_loggedPct + 10) {
      s_loggedPct = pct - pct % 10;
      LOGI("ota: %s %u%% (%lu B)", kind, (unsigned)pct, (unsigned long)s_written.load());
    }
    const int32_t silent = (int32_t)(millis() - s_lastDataMs.load());
    if (silent > (int32_t)kStallMs) {
      // The upload handler never finished (connection lost without a
      // disconnect callback). Do not touch Update from here: just restart,
      // which discards the partial image.
      s_loggedEnd = true;
      s_endedAtMs = nowMs;
      EventLog::add('e', "ota_fail", "%s upload stalled %lu ms", kind, (unsigned long)silent);
    }
  }

  if (phase == kEnded && !s_loggedEnd) {
    s_loggedEnd = true;
    s_endedAtMs = nowMs;
    char res[sizeof(s_result)];
    portENTER_CRITICAL(&s_mux);
    memcpy(res, s_result, sizeof(res));
    portEXIT_CRITICAL(&s_mux);
    if (s_ok.load()) {
      EventLog::add('i', "ota_ok", "%s update ok, %lu B", kind, (unsigned long)s_written.load());
    } else {
      EventLog::add('e', "ota_fail", "%s: %s", kind, res);
    }
  }

  if (s_loggedEnd && !s_rebootAsked && (uint32_t)(nowMs - s_endedAtMs) >= kRebootAfterMs) {
    s_rebootAsked = true;
    appRequestReboot(100, "ota");
  }
}

bool active() { return s_phase.load() != kIdle; }

uint8_t progressPct() { return s_progress.load(); }

}  // namespace Ota

#else  // !ENABLE_OTA

namespace Ota {
void attach(AsyncWebServer& server) { (void)server; }
void setPassword(const char* apPass) { (void)apPass; }
void tick(uint32_t nowMs) { (void)nowMs; }
bool active() { return false; }
uint8_t progressPct() { return 0; }
}  // namespace Ota

#endif  // ENABLE_OTA
