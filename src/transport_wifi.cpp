// WiFi access point, HTTP server, WebSocket, static panel from LittleFS.
//
// Threading (see shared_state.h):
//   async_tcp task  WS / HTTP callbacks: parse and validate, push into
//                   CmdQueue, answer from the caches. Never waits for the loop.
//   loop task       begin(), tick(), sendTo(), broadcast(), publish().
// State shared between the two: the WS client table (portMUX), the caches
// (FreeRTOS mutex) and a few atomics. Sends from the loop go through the
// library's own locked API (AsyncWebSocket::text / textAll / close).
#include "transport_wifi.h"

#include "config.h"

#if ENABLE_WIFI

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>

#if ENABLE_CAPTIVE
#include <DNSServer.h>
#endif

#include "app.h"
#include "cmd_parser.h"
#include "cmd_queue.h"
#include "frames.h"
#include "log.h"
#include "ota.h"
#include "protocol.h"
#include "stop_seq_guard.h"

namespace {

using TransportWifi::Cache;

constexpr const char* kJson = "application/json";
constexpr const char* kPortalUrl = "http://" AP_IP_STR "/";
// Tracked WS clients. lwIP allows CONFIG_LWIP_MAX_ACTIVE_TCP (16) sockets in
// total, so in practice the table never fills; a client that does not fit is
// closed from tick().
constexpr uint8_t kWsSlots = 16;
constexpr uint16_t kWsKeep = 4;  // cleanupClients(): oldest closed above this count
constexpr uint32_t kSilentCheckMs = 100;
constexpr uint32_t kHousekeepMs = 1000;

// Built-in page for a device whose LittleFS has no panel yet.
const char kFallbackPage[] = R"HTML(<!doctype html>
<html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turbina: serwis</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}
body{margin:0;background:#131518;color:#d5d9de;font:15px/1.45 system-ui,sans-serif}
.tape{height:10px;background:repeating-linear-gradient(-45deg,#f2b705 0 12px,#131518 12px 24px)}
main{max-width:520px;margin:0 auto;padding:16px}
h1{font-size:13px;letter-spacing:.16em;text-transform:uppercase;color:#f2b705;margin:6px 0 14px}
section{background:#1b1e22;border:1px solid #30353b;border-radius:3px;padding:14px;margin-bottom:12px}
h2{font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:#8b939c;margin:0 0 8px}
p{margin:0 0 8px}
code{display:block;background:#0d0f11;border:1px solid #2a2e33;padding:8px;margin:6px 0;font:13px ui-monospace,monospace;color:#9fd8a0;overflow-x:auto;white-space:nowrap}
label{display:block;font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:#8b939c;margin:10px 0 4px}
input,select,button{width:100%;font:inherit;color:inherit;background:#0d0f11;border:1px solid #3a4047;border-radius:3px;padding:10px}
button{min-height:56px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;cursor:pointer}
#go{margin-top:14px;background:#f2b705;border-color:#f2b705;color:#131518}
#go:disabled{opacity:.45}
#stop{background:#b71c1c;border-color:#e53935;color:#fff}
.bar{height:8px;margin-top:12px;background:#0d0f11;border:1px solid #30353b}
#bar{height:100%;width:0;background:#f2b705;transition:width .2s}
#msg{margin-top:8px;min-height:1.4em;font:13px ui-monospace,monospace}
</style></head><body><div class="tape"></div><main>
<h1>Turbina / serwis</h1>
<section><h2>Brak panelu sterowania</h2>
<p>W pamięci LittleFS nie ma plików panelu (katalog <b>data/</b>). Wgraj je kablem USB:</p>
<code>pio run -e esp32dev -t uploadfs</code>
<p>albo plikiem <b>littlefs.bin</b> z tej strony. Plik powstaje poleceniem:</p>
<code>pio run -e esp32dev -t buildfs</code>
<p>i leży w <b>.pio/build/esp32dev/littlefs.bin</b>. Dla ESP32-S3 użyj -e esp32-s3-devkitc-1.</p></section>
<section><h2>Wgrywanie</h2><form id="f">
<label for="type">Typ pliku</label>
<select id="type"><option value="fs">Panel (littlefs.bin)</option><option value="fw">Firmware (firmware.bin)</option></select>
<label for="file">Plik</label><input id="file" type="file" accept=".bin">
<label for="pw">Hasło sieci WiFi (AP)</label><input id="pw" type="password" autocomplete="current-password">
<button id="go" type="submit">Wgraj</button>
<div class="bar"><div id="bar"></div></div><div id="msg"></div></form></section>
<section><h2>Awaria</h2><button id="stop" type="button">Stop wszystko</button></section>
</main><script>
var $=function(i){return document.getElementById(i)},msg=function(t){$('msg').textContent=t};
var errs={'motors running':'silniki pracują, zatrzymaj je','busy':'trwa inna aktualizacja','firmware image sent as fs':'to jest plik firmware, wybierz typ Firmware','not a firmware image':'to nie jest plik firmware, wybierz typ Panel','empty file':'pusty plik','no file in request':'brak pliku','incomplete upload':'wysyłanie przerwane'};
$('f').onsubmit=function(e){e.preventDefault();var file=$('file').files[0];
if(!file){msg('Wybierz plik.');return}
var fd=new FormData();fd.append('file',file,file.name);
var x=new XMLHttpRequest();x.open('POST','/update?type='+$('type').value);
x.setRequestHeader('Authorization','Basic '+btoa(')HTML" OTA_USER R"HTML(:'+$('pw').value));
x.upload.onprogress=function(ev){if(ev.lengthComputable){var p=Math.round(ev.loaded*100/ev.total);$('bar').style.width=p+'%';msg('Wysyłanie: '+p+' %')}};
x.onload=function(){var r={};try{r=JSON.parse(x.responseText)}catch(_){}
if(x.status==200){$('bar').style.width='100%';msg('Gotowe. Restart urządzenia, strona odświeży się za 10 s.');setTimeout(function(){location.reload()},10000);return}
$('go').disabled=false;
if(x.status==401)msg('Złe hasło.');else msg('Błąd '+x.status+': '+(errs[r.error]||'zapis obrazu nie powiódł się'))};
x.onerror=function(){$('go').disabled=false;msg('Błąd połączenia.')};
$('go').disabled=true;msg('Wysyłanie...');x.send(fd)};
$('stop').onclick=function(){fetch('/api/cmd',{method:'POST',body:'{"t":"stop"}'}).then(function(){msg('Wysłano STOP.')},function(){msg('Brak połączenia.')})};
</script></body></html>
)HTML";

AsyncWebServer s_server(HTTP_PORT);
AsyncWebSocket s_ws("/ws");
#if ENABLE_CAPTIVE
DNSServer s_dns;
#endif

bool s_started = false;  // loop only
bool s_fsOk = false;
bool s_captive = false;
uint8_t s_txQdbm = WIFI_TX_QDBM;

// --- WS client table (async_tcp writes, loop reads) -------------------------
struct WsSlot {
  uint32_t id;
  uint32_t lastRxMs;
  bool used;
  bool closing;  // close() already requested for silence
};
WsSlot s_slots[kWsSlots] = {};
portMUX_TYPE s_wsMux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<uint8_t> s_wsCount{0};
// A client refused at connect. Closed later from tick(): closing inside the
// connect event could erase it while the library is still adding it.
std::atomic<uint32_t> s_refusedId{0};  // WS ids start at 1

bool slotAdd(uint32_t id, uint32_t now) {
  bool ok = false;
  portENTER_CRITICAL(&s_wsMux);
  for (WsSlot& s : s_slots) {
    if (!s.used) {
      s.id = id;
      s.lastRxMs = now;
      s.used = true;
      s.closing = false;
      s_wsCount.fetch_add(1);
      ok = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_wsMux);
  return ok;
}

bool slotRemove(uint32_t id) {
  bool found = false;
  portENTER_CRITICAL(&s_wsMux);
  for (WsSlot& s : s_slots) {
    if (s.used && s.id == id) {
      s.used = false;
      s_wsCount.fetch_sub(1);
      found = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_wsMux);
  return found;
}

bool slotFind(uint32_t id) {
  bool found = false;
  portENTER_CRITICAL(&s_wsMux);
  for (const WsSlot& s : s_slots) {
    if (s.used && s.id == id) {
      found = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_wsMux);
  return found;
}

void slotTouch(uint32_t id, uint32_t now) {
  portENTER_CRITICAL(&s_wsMux);
  for (WsSlot& s : s_slots) {
    if (s.used && s.id == id) {
      s.lastRxMs = now;
      break;
    }
  }
  portEXIT_CRITICAL(&s_wsMux);
}

// --- Stations (loop only, refresh requested by the WiFi event task) ----------
uint8_t s_stations = 0;
bool s_hasRssi = false;
int8_t s_rssi = 0;
std::atomic<bool> s_staDirty{true};
uint32_t s_lastStaMs = 0;
uint32_t s_lastCleanupMs = 0;
uint32_t s_lastSilentMs = 0;

// --- Caches for HTTP (loop writes, async_tcp reads) -------------------------
constexpr size_t kCacheSmall = 1024;
constexpr size_t kCacheLog = 2304;  // a full 20-item log is about 2.2 kB
char s_cacheState[kCacheSmall];
char s_cacheCfg[kCacheSmall];
char s_cacheInfo[kCacheSmall];
char s_cacheLog[kCacheLog];
struct CacheBuf {
  char* data;
  uint16_t cap;
  uint16_t len;
  bool warned;
};
CacheBuf s_caches[4] = {
    {s_cacheState, sizeof(s_cacheState), 0, false},
    {s_cacheCfg, sizeof(s_cacheCfg), 0, false},
    {s_cacheInfo, sizeof(s_cacheInfo), 0, false},
    {s_cacheLog, sizeof(s_cacheLog), 0, false},
};
SemaphoreHandle_t s_cacheMutex = nullptr;
StaticSemaphore_t s_cacheMutexBuf;

// Copy a cache into `out` (NUL-terminated). 0 if empty or busy.
size_t copyCache(Cache which, char* out, size_t cap) {
  const uint8_t i = static_cast<uint8_t>(which);
  if (i >= 4 || !s_cacheMutex || cap == 0) return 0;
  if (xSemaphoreTake(s_cacheMutex, pdMS_TO_TICKS(20)) != pdTRUE) return 0;
  const CacheBuf& c = s_caches[i];
  size_t n = 0;
  if (c.len > 0 && c.len < cap) {
    memcpy(out, c.data, c.len);
    n = c.len;
  }
  xSemaphoreGive(s_cacheMutex);
  out[n] = '\0';
  return n;
}

// --- HTTP helpers -----------------------------------------------------------
void sendJson(AsyncWebServerRequest* req, int code, const char* body) {
  AsyncWebServerResponse* r = req->beginResponse(code, kJson, body);  // copies body
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}

void sendErr(AsyncWebServerRequest* req, proto::Cmd of, proto::Err e, const char* field, const char* msg,
             bool hasId, uint32_t id) {
  char out[192];
  if (!buildErr(of, e, field, msg, hasId, id, out, sizeof(out))) {
    strlcpy(out, "{\"t\":\"err\",\"of\":\"unknown\",\"code\":\"busy\"}", sizeof(out));
  }
  sendJson(req, 200, out);
}

void sendCache(AsyncWebServerRequest* req, Cache which, const Command& cmd) {
  char buf[kCacheLog + 1];
  if (copyCache(which, buf, sizeof(buf)) == 0) {
    sendErr(req, cmd.type, proto::Err::Busy, nullptr, "not ready", cmd.hasId, cmd.id);
    return;
  }
  sendJson(req, 200, buf);
}

// Send order of HTTP commands (async_tcp only, see stop_seq_guard.h).
StopSeqGuard s_seqGuard;

uint32_t remoteIpv4(AsyncWebServerRequest* req) {
  AsyncClient* c = req->client();
  return c ? c->getRemoteAddress() : 0;
}

// GET /api/state
void handleState(AsyncWebServerRequest* req) {
  char buf[kCacheSmall + 1];
  if (copyCache(Cache::State, buf, sizeof(buf)) == 0) {
    char out[160];
    buildErr(proto::Cmd::None, proto::Err::Busy, nullptr, "no telemetry yet", false, 0, out, sizeof(out));
    sendJson(req, 503, out);
    return;
  }
  sendJson(req, 200, buf);
}

// POST /api/cmd: body collected here (request->_tempObject, freed by the request)
struct CmdBody {
  uint16_t len;
  bool overflow;
  char data[proto::kMaxCmdLen + 1];
};

void onCmdBody(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  (void)index;
  (void)total;
  CmdBody* b = static_cast<CmdBody*>(req->_tempObject);
  if (!b) {
    b = static_cast<CmdBody*>(calloc(1, sizeof(CmdBody)));
    if (!b) return;
    req->_tempObject = b;
  }
  if (b->overflow || len == 0) return;
  // Keep what fits: the start of an oversized frame still tells its "t".
  const size_t room = proto::kMaxCmdLen - b->len;
  const size_t n = len < room ? len : room;
  memcpy(b->data + b->len, data, n);
  b->len = static_cast<uint16_t>(b->len + n);
  b->data[b->len] = '\0';
  if (len > room) b->overflow = true;
}

void handleCmd(AsyncWebServerRequest* req) {
  const CmdBody* b = static_cast<const CmdBody*>(req->_tempObject);
  const char* body = "";
  size_t len = 0;
  bool tooLong = req->contentLength() > proto::kMaxCmdLen || (b && b->overflow);
  if (!tooLong) {
    if (b && b->len > 0) {
      body = b->data;
      len = b->len;
    } else if (const AsyncWebParameter* p = req->getParam("body", true)) {
      // application/x-www-form-urlencoded: the library parsed the JSON text
      // into a parameter named "body" instead of calling onCmdBody.
      body = p->value().c_str();
      len = p->value().length();
      tooLong = len > proto::kMaxCmdLen;
    }
  }
  if (tooLong) {
    const proto::Cmd of = b ? peekCommandType(b->data, b->len) : proto::Cmd::None;
    sendErr(req, of, proto::Err::TooLong, nullptr, nullptr, false, 0);
    return;
  }

  Command cmd;
  const ParseResult pr = parseCommand(body, len, cmd);
  if (pr.err != proto::Err::None) {
    sendErr(req, cmd.type, pr.err, pr.field, nullptr, cmd.hasId, cmd.id);
    return;
  }

  // Every request is its own TCP connection: a START delayed on the air
  // must not run after the STOP the user pressed later.
  const uint32_t ip = remoteIpv4(req);
  if (!s_seqGuard.accept(ip, cmd, millis())) {
    LOGW("http: stale %s (seq %lu) dropped", proto::cmdName(cmd.type), (unsigned long)cmd.seq);
    sendErr(req, cmd.type, proto::Err::Stale, nullptr, nullptr, cmd.hasId, cmd.id);
    return;
  }

  char out[160];
  if (!CmdQueue::pushCommand(proto::Source::Http, ip, body, len)) {
    if (cmd.type == proto::Cmd::Stop) {
      // The estop flag is raised even with a full queue: the stop happens.
      buildAck(cmd.type, cmd.hasId, cmd.id, out, sizeof(out));
      sendJson(req, 200, out);
    } else {
      sendErr(req, cmd.type, proto::Err::Busy, nullptr, "queue full", cmd.hasId, cmd.id);
    }
    return;
  }

  switch (cmd.type) {
    case proto::Cmd::Ping:
      buildPong(cmd.hasId, cmd.id, millis(), out, sizeof(out));
      sendJson(req, 200, out);
      return;
    case proto::Cmd::Cfg:
      if (cmd.cfgGet && !cmd.cfgReset && cmd.cfgMask == 0) {
        sendCache(req, Cache::Cfg, cmd);
        return;
      }
      break;
    case proto::Cmd::Info:
      sendCache(req, Cache::Info, cmd);
      return;
    case proto::Cmd::Log:
      sendCache(req, Cache::Log, cmd);
      return;
    default:
      break;
  }
  buildAck(cmd.type, cmd.hasId, cmd.id, out, sizeof(out));
  sendJson(req, 200, out);
}

void handleFallback(AsyncWebServerRequest* req) {
  AsyncWebServerResponse* r = req->beginResponse(200, "text/html; charset=utf-8",
                                                 reinterpret_cast<const uint8_t*>(kFallbackPage),
                                                 sizeof(kFallbackPage) - 1);  // served from flash, no copy
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}

void handleNotFound(AsyncWebServerRequest* req) {
  if (s_captive) {
    req->redirect(kPortalUrl);
    return;
  }
  req->send(404, "text/plain", "404: nie ma takiej strony");
}

bool isLongCacheAsset(AsyncWebServerRequest* req) {
  const String& u = req->url();
  return u.endsWith(".png") || u.endsWith(".svg") || u.endsWith(".ico") || u.endsWith(".webp");
}

// Captive mode: any request addressed to another host goes to the panel.
bool isForeignHost(AsyncWebServerRequest* req) {
  const String& h = req->host();
  if (h.length() == 0) return false;
  return !(h == AP_IP_STR || h.startsWith(AP_IP_STR ":"));
}

// --- WebSocket ----------------------------------------------------------------
void wsReply(AsyncWebSocketClient* client, const char* data, size_t len) {
  if (len > 0) client->text(data, len);
}

void wsOnConnect(AsyncWebSocketClient* client) {
  // Drop frames for a slow client instead of closing it (3.12.1 default is
  // already false; set explicitly so a library default change cannot bite).
  client->setCloseClientOnQueueFull(false);
  if (AsyncClient* tcp = client->client()) tcp->setNoDelay(true);  // no Nagle on tiny frames

  const uint32_t id = client->id();
  if (!slotAdd(id, millis())) {
    LOGW("ws #%lu refused: %u clients", (unsigned long)id, (unsigned)kWsSlots);
    s_refusedId.store(id);
    return;
  }
  CmdQueue::pushEvent(MsgKind::Connected, proto::Source::Ws, id);
  LOGI("ws #%lu connected", (unsigned long)id);
}

void wsOnDisconnect(uint32_t id) {
  if (!slotRemove(id)) return;  // refused at connect, never announced
  CmdQueue::pushEvent(MsgKind::Disconnected, proto::Source::Ws, id);
  LOGI("ws #%lu disconnected", (unsigned long)id);
}

// Unfragmented text frames that arrive in pieces (they straddled TCP
// segments), one partial frame per client. WS data events only run in the
// async_tcp task, one at a time, so this needs no lock. WS ids are never
// reused: an entry left behind by a client that went away is simply taken
// over later (it is not freed on disconnect, which may run in the loop).
struct WsRx {
  uint32_t id;  // 0 = free
  uint32_t startMs;
  uint16_t have;
  uint16_t total;
  char buf[proto::kMaxCmdLen + 1];
};
constexpr uint8_t kRxSlots = 4;
WsRx s_rx[kRxSlots] = {};

WsRx* rxFind(uint32_t id) {
  for (WsRx& r : s_rx) {
    if (r.id == id) return &r;
  }
  return nullptr;
}

// Entry for a new partial frame of client id: its own, a free one, or the
// one whose frame started longest ago.
WsRx* rxTake(uint32_t id, uint32_t now) {
  WsRx* r = rxFind(id);
  if (!r) {
    r = &s_rx[0];
    for (WsRx& c : s_rx) {
      if (c.id == 0) {
        r = &c;
        break;
      }
      if ((int32_t)(c.startMs - r->startMs) < 0) r = &c;
    }
  }
  r->id = id;
  r->startMs = now;
  r->have = 0;
  return r;
}

void wsOnData(AsyncWebSocketClient* client, const AwsFrameInfo* info, const uint8_t* data, size_t len) {
  const uint32_t id = client->id();
  if (id == s_refusedId.load()) return;  // being closed, never announced to the loop
  slotTouch(id, millis());

  const char* text = reinterpret_cast<const char*>(data);
  char out[192];

  // Commands are single-frame text messages. Continuation frames and
  // binary data are ignored.
  if (info->opcode != WS_TEXT || !info->final) return;

  // Too long: answer once, on the first piece of the frame.
  if (info->len > proto::kMaxCmdLen) {
    if (info->index == 0) {
      const size_t n = buildErr(peekCommandType(text, len), proto::Err::TooLong, nullptr, nullptr, false, 0, out,
                                sizeof(out));
      wsReply(client, out, n);
    }
    return;
  }

  if (info->index != 0 || len != info->len) {
    // A piece of a frame: collect it per client, act once it is complete.
    WsRx* rx;
    if (info->index == 0) {
      rx = rxTake(id, millis());
      rx->total = static_cast<uint16_t>(info->len);
    } else {
      rx = rxFind(id);
      if (!rx) return;  // its start was lost (slot taken over)
      if (rx->have != info->index) {
        rx->id = 0;  // a gap: drop the frame
        return;
      }
    }
    if ((size_t)rx->have + len > rx->total) {
      rx->id = 0;
      return;
    }
    memcpy(rx->buf + rx->have, data, len);
    rx->have = static_cast<uint16_t>(rx->have + len);
    if (rx->have < rx->total) return;
    rx->buf[rx->have] = '\0';
    rx->id = 0;  // free again; buf stays valid until the next data event
    text = rx->buf;
    len = rx->have;
  }

  if (CmdQueue::pushCommand(proto::Source::Ws, id, text, len)) return;

  // Queue full: answer here so the client is not left waiting.
  Command cmd;
  parseCommand(text, len, cmd);  // only for "of" and "id"
  size_t n;
  if (cmd.type == proto::Cmd::Stop) {
    n = buildAck(cmd.type, cmd.hasId, cmd.id, out, sizeof(out));  // estop flag already raised
  } else {
    n = buildErr(cmd.type, proto::Err::Busy, nullptr, "queue full", cmd.hasId, cmd.id, out, sizeof(out));
  }
  wsReply(client, out, n);
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data,
               size_t len) {
  (void)server;
  if (!client) return;
  switch (type) {
    case WS_EVT_CONNECT:
      wsOnConnect(client);
      break;
    case WS_EVT_DISCONNECT:
      wsOnDisconnect(client->id());
      break;
    case WS_EVT_DATA:
      if (arg) wsOnData(client, static_cast<const AwsFrameInfo*>(arg), data, len);
      break;
    default:
      break;
  }
}

// Close clients that sent nothing for WS_SILENT_CLOSE_MS (the panel pings
// every 300 ms). Dead TCP peers are also caught by AsyncTCP's ack timeout.
void closeSilentClients() {
  uint32_t ids[kWsSlots];
  uint8_t n = 0;
  const uint32_t now = millis();
  portENTER_CRITICAL(&s_wsMux);
  for (WsSlot& s : s_slots) {
    if (!s.used || s.closing) continue;
    // signed: lastRxMs may be a hair newer than `now` (written by async_tcp)
    if ((int32_t)(now - s.lastRxMs) > (int32_t)WS_SILENT_CLOSE_MS) {
      s.closing = true;
      ids[n++] = s.id;
    }
  }
  portEXIT_CRITICAL(&s_wsMux);
  for (uint8_t i = 0; i < n; i++) {
    LOGI("ws #%lu silent > %u ms, closing", (unsigned long)ids[i], (unsigned)WS_SILENT_CLOSE_MS);
    s_ws.close(ids[i], 1001, "silent");
  }
}

void refreshStations() {
  wifi_sta_list_t list;
  memset(&list, 0, sizeof(list));
  uint8_t n = 0;
  bool has = false;
  int8_t weakest = 0;
  if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
    n = static_cast<uint8_t>(list.num);
    for (int i = 0; i < list.num; i++) {
      const int8_t r = list.sta[i].rssi;
      if (!has || r < weakest) weakest = r;
      has = true;
    }
  }
  if (n != s_stations) LOGI("wifi: %u station(s)", (unsigned)n);
  s_stations = n;
  s_hasRssi = has;
  s_rssi = weakest;
}

// WiFi event task: only flag a refresh, the loop does the work.
void onWifiEvent(arduino_event_id_t event) {
  (void)event;
  s_staDirty.store(true);
}

void registerCaptive(const IPAddress& ip) {
#if ENABLE_CAPTIVE
  if (!s_captive) return;
  if (s_dns.start(53, "*", ip)) {
    LOGI("captive: DNS answers %s for every name", AP_IP_STR);
  } else {
    LOGE("captive: DNS server failed to start");
  }
  WiFi.AP.enableDhcpCaptivePortal();  // DHCP option 114 (RFC 8910)

  auto toPortal = [](AsyncWebServerRequest* req) { req->redirect(kPortalUrl); };
  s_server.on("/*", HTTP_ANY, toPortal).setFilter(isForeignHost);
  static const char* const kProbes[] = {"/generate_204", "/gen_204", "/hotspot-detect.html", "/connecttest.txt",
                                        "/ncsi.txt"};
  for (const char* p : kProbes) s_server.on(p, HTTP_ANY, toPortal);
#else
  (void)ip;
#endif
}

}  // namespace

namespace TransportWifi {

void begin(const Settings& s) {
  if (s_started) return;
  if (!s_cacheMutex) s_cacheMutex = xSemaphoreCreateMutexStatic(&s_cacheMutexBuf);

  // Filesystem (formats an unformatted partition; the fallback page then
  // offers the upload of littlefs.bin).
  s_fsOk = LittleFS.begin(true);
  const bool hasPanel = s_fsOk && (LittleFS.exists("/index.html.gz") || LittleFS.exists("/index.html"));
  if (!s_fsOk) {
    LOGE("littlefs: mount failed, serving the built-in page");
  } else if (!hasPanel) {
    LOGW("littlefs: no index.html(.gz), serving the built-in page (pio run -t uploadfs)");
  }

  // Access point
  const char* pass = s.apPass;
  const size_t passLen = strlen(pass);
  if (passLen < 8 || passLen > 63) {
    LOGW("ap: stored password unusable (%u chars), using the default", (unsigned)passLen);
    pass = AP_PASSWORD;
  }
  const uint8_t chan = (s.apChan >= 1 && s.apChan <= 13) ? s.apChan : AP_CHANNEL;
  s_captive = ENABLE_CAPTIVE && s.captive;
  IPAddress ip;
  ip.fromString(AP_IP_STR);
  const IPAddress mask(255, 255, 255, 0);

#if ENABLE_OTA
  Ota::setPassword(pass);  // the one the AP really uses until the next restart
#endif

  WiFi.persistent(false);  // settings live in our own NVS namespace
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(appDeviceName(), pass, chan, 0, AP_MAX_CLIENTS)) LOGE("ap: softAP failed");
  // DNS offered by DHCP only in captive mode (then our DNS server answers).
  if (!WiFi.softAPConfig(ip, ip, mask, IPAddress((uint32_t)0), s_captive ? ip : IPAddress((uint32_t)0))) {
    LOGE("ap: IP config failed");
  }
  WiFi.softAPbandwidth(WIFI_BW_HT20);  // narrower channel: more robust in a crowded band
  s_started = true;
  s_txQdbm = s.txPower;
  setTxPower(s_txQdbm);
  const esp_err_t psErr = esp_wifi_set_ps(WIFI_PS_NONE);
  if (psErr != ESP_OK) LOGW("wifi: power save off failed: %d", (int)psErr);
  WiFi.onEvent(onWifiEvent, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(onWifiEvent, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  LOGI("ap: %s on channel %u, %s", appDeviceName(), (unsigned)chan, AP_IP_STR);

  // HTTP. Handlers are matched in registration order: WS and the API first,
  // so they never pay for LittleFS lookups of the static handler.
  s_ws.onEvent(onWsEvent);
  s_server.addHandler(&s_ws);
  s_server.on("/api/state", HTTP_GET, handleState);
  s_server.on("/api/cmd", HTTP_POST, handleCmd, nullptr, onCmdBody);
#if ENABLE_OTA
  Ota::attach(s_server);
#endif
  registerCaptive(ip);
  if (s_fsOk) {
    // A request for /x serves /x.gz with Content-Encoding: gzip when only the
    // .gz exists (AsyncStaticWebHandler tries the .gz first). "/" maps to
    // /index.html(.gz). gz files get an ETag from their CRC, so no-cache
    // costs a 304 round trip, not a download.
    s_server.serveStatic("/", LittleFS, "/", "max-age=86400").setFilter(isLongCacheAsset);
    s_server.serveStatic("/", LittleFS, "/", "no-cache").setDefaultFile("index.html");
  }
  // Reached only when the static handler found no index file.
  s_server.on("/", HTTP_GET, handleFallback);
  s_server.on("/index.html", HTTP_GET, handleFallback);
  s_server.onNotFound(handleNotFound);
  s_server.begin();
}

void tick(uint32_t nowMs) {
  if (!s_started) return;
  if (const uint32_t refused = s_refusedId.exchange(0)) s_ws.close(refused, 1013, "too many clients");
  if ((uint32_t)(nowMs - s_lastSilentMs) >= kSilentCheckMs) {
    s_lastSilentMs = nowMs;
    closeSilentClients();
  }
  if ((uint32_t)(nowMs - s_lastCleanupMs) >= kHousekeepMs) {
    s_lastCleanupMs = nowMs;
    s_ws.cleanupClients(kWsKeep);
  }
  if (s_staDirty.exchange(false) || (uint32_t)(nowMs - s_lastStaMs) >= kHousekeepMs) {
    s_lastStaMs = nowMs;
    refreshStations();
  }
}

void sendTo(uint32_t clientId, const char* data, size_t len) {
  if (!s_started || !data || len == 0) return;
  s_ws.text(clientId, data, len);
}

void broadcast(const char* data, size_t len) {
  if (!s_started || !data || len == 0 || s_wsCount.load() == 0) return;
  // One shared buffer for every client. A client whose queue is full
  // (WS_MAX_QUEUED_MESSAGES) skips this frame: closeWhenFull is off.
  s_ws.textAll(data, len);
}

uint8_t wsClients() { return s_wsCount.load(); }

bool hasClient(uint32_t clientId) { return slotFind(clientId); }

uint8_t stations() { return s_stations; }

bool rssi(int8_t& out) {
  if (!s_hasRssi) return false;
  out = s_rssi;
  return true;
}

void setTxPower(uint8_t qdbm) {
  s_txQdbm = qdbm;
  if (!s_started) return;  // applied by begin()
  const uint8_t snapped = snapTxPower(qdbm);
  if (!WiFi.setTxPower(static_cast<wifi_power_t>(snapped))) {
    LOGW("wifi: tx power %u failed", (unsigned)snapped);
  }
}

void publish(Cache which, const char* data, size_t len) {
  const uint8_t i = static_cast<uint8_t>(which);
  if (i >= 4 || !data || !s_cacheMutex) return;
  CacheBuf& c = s_caches[i];
  if (len >= c.cap) {
    if (!c.warned) LOGW("http cache %u: frame of %u B does not fit", (unsigned)i, (unsigned)len);
    c.warned = true;
    return;
  }
  // Short wait: an HTTP handler holds the mutex only for a memcpy.
  if (xSemaphoreTake(s_cacheMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  memcpy(c.data, data, len);
  c.data[len] = '\0';
  c.len = static_cast<uint16_t>(len);
  xSemaphoreGive(s_cacheMutex);
}

}  // namespace TransportWifi

#else  // !ENABLE_WIFI

namespace TransportWifi {
void begin(const Settings& s) { (void)s; }
void tick(uint32_t nowMs) { (void)nowMs; }
void sendTo(uint32_t clientId, const char* data, size_t len) {
  (void)clientId;
  (void)data;
  (void)len;
}
void broadcast(const char* data, size_t len) {
  (void)data;
  (void)len;
}
uint8_t wsClients() { return 0; }
bool hasClient(uint32_t clientId) {
  (void)clientId;
  return false;
}
uint8_t stations() { return 0; }
bool rssi(int8_t& out) {
  (void)out;
  return false;
}
void setTxPower(uint8_t qdbm) { (void)qdbm; }
void publish(Cache which, const char* data, size_t len) {
  (void)which;
  (void)data;
  (void)len;
}
}  // namespace TransportWifi

#endif  // ENABLE_WIFI
