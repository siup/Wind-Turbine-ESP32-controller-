// Serial console: stage 1 bench commands and a JSON pass-through.
//
// Non-blocking: tick() reads what is already in the RX buffer (bounded per
// call), lines end with \n, \r\n or a lone \r. Shorthands become protocol
// frames pushed to CmdQueue as Source::Console, so they take exactly the same
// path (parser, failsafe, limits) as WebSocket and BLE frames. Output is
// written only when the TX buffer has room, so a slow or absent terminal can
// never stall the control loop (lines are dropped instead).
//
// Keepalive frames carry a reserved "id"; their pong / ack replies are
// swallowed in print() so `ka on` does not flood the terminal.

#include "serial_console.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

#if ENABLE_SERIAL_CONSOLE

#include "battery.h"
#include "cmd_parser.h"
#include "cmd_queue.h"
#include "commands.h"
#include "frames.h"
#include "protocol.h"
#include "yaw.h"

namespace {

constexpr size_t kLineMax = 300;          // longest accepted input line
constexpr size_t kReadBudget = 256;       // max bytes consumed per tick
constexpr uint32_t kKeepAlivePeriodMs = 200;
constexpr uint32_t kTelPeriodMs = 500;
constexpr uint32_t kHoldStartGraceMs = 300;  // time for a yl/yr to start the motor
constexpr uint32_t kHoldMaxMs = 60000;       // ka feeds one yaw move at most this long
constexpr uint32_t kKaErrGapMs = 2000;       // errors caused by ka frames: 1 line per 2 s

// Reserved id of keepalive frames. Keep both spellings in sync.
#define KA_ID_STR "4242424242"
constexpr char kKaNeedle[] = "\"id\":" KA_ID_STR;

const char kPingFrame[] = "{\"t\":\"ping\",\"id\":" KA_ID_STR "}";
const char kHoldLeftFrame[] = "{\"t\":\"yaw\",\"hold\":\"left\",\"id\":" KA_ID_STR "}";
const char kHoldRightFrame[] = "{\"t\":\"yaw\",\"hold\":\"right\",\"id\":" KA_ID_STR "}";

const char* const kHelp[] = {
    "# Konsola turbiny, komendy:",
    "#   r<0-100>    rotor, prędkość w % (np. r50)",
    "#   rb          rotor stop z hamowaniem",
    "#   yl / yr     yaw w lewo / w prawo (trzymaj: ka on albo powtarzaj)",
    "#   ys          yaw stop",
    "#   s           stop wszystkiego",
    "#   g<stopnie>  yaw goto (tylko AS5600), np. g135",
    "#   b           bateria: mV, mV na ADC, %, flagi",
    "#   tel on|off  telemetria co 500 ms",
    "#   ka on|off   podtrzymanie: ping + yaw hold co 200 ms",
    "#   {...}       dowolna ramka JSON protokołu",
    "#   ?           ta pomoc",
};
constexpr size_t kHelpCount = sizeof(kHelp) / sizeof(kHelp[0]);

// Line reader
char g_line[kLineMax + 1];
size_t g_lineLen = 0;
bool g_overflow = false;  // current line is longer than kLineMax: discard it
bool g_afterCr = false;   // swallow the \n of a \r\n pair

// Modes
bool g_tel = false;
uint32_t g_telMs = 0;
bool g_ka = false;
uint32_t g_kaMs = 0;
uint32_t g_kaErrMs = 0;
bool g_kaErrShown = false;

// Last yaw command from this console was yl / yr: ka keeps feeding the
// deadman with `hold` in that direction until a stop is seen.
int8_t g_holdDir = 0;  // -1 left, +1 right, 0 none
bool g_holdSeenMoving = false;
uint32_t g_holdSetMs = 0;

size_t g_helpIdx = kHelpCount;  // next help line to print (== count: idle)

char g_out[320];  // output frames (telemetry, battery, errors, notes)
char g_cmd[96];   // shorthand -> JSON frame

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

bool txRoom(size_t len) {
  const int room = Serial.availableForWrite();
  return room > 0 && (size_t)room >= len + 1;
}

// One line, only if it fits in the TX buffer right now.
void writeLine(const char* data, size_t len) {
  if (!data || !len || !txRoom(len)) return;
  Serial.write(reinterpret_cast<const uint8_t*>(data), len);
  Serial.write(static_cast<uint8_t>('\n'));
}

void note(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void note(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(g_out, sizeof(g_out), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  writeLine(g_out, (size_t)n < sizeof(g_out) ? (size_t)n : sizeof(g_out) - 1);
}

void printErr(proto::Cmd of, proto::Err e, const char* field) {
  const size_t n = buildErr(of, e, field, nullptr, false, 0, g_out, sizeof(g_out));
  if (n) writeLine(g_out, n);
}

// Find needle in data[0..len) (data need not be NUL-terminated).
const char* findIn(const char* data, size_t len, const char* needle, size_t nlen) {
  if (nlen == 0 || len < nlen) return nullptr;
  for (size_t i = 0; i + nlen <= len; i++) {
    if (data[i] == needle[0] && memcmp(data + i, needle, nlen) == 0) return data + i;
  }
  return nullptr;
}

// Replies to keepalive frames: pong / ack are noise, errors are shown at
// most once per kKaErrGapMs.
bool swallowKeepAliveReply(const char* data, size_t len) {
  const size_t nlen = sizeof(kKaNeedle) - 1;
  const char* p = findIn(data, len, kKaNeedle, nlen);
  if (!p) return false;
  const size_t after = (size_t)(p - data) + nlen;
  if (after < len && isdigit((unsigned char)data[after])) return false;  // a longer id

  static const char kErrPrefix[] = "{\"t\":\"err\"";
  const bool isErr = len >= sizeof(kErrPrefix) - 1 && memcmp(data, kErrPrefix, sizeof(kErrPrefix) - 1) == 0;
  if (!isErr) return true;
  const uint32_t now = millis();
  if (g_kaErrShown && (uint32_t)(now - g_kaErrMs) < kKaErrGapMs) return true;
  g_kaErrShown = true;
  g_kaErrMs = now;
  return false;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void clearHold() {
  g_holdDir = 0;
  g_holdSeenMoving = false;
}

// Push one frame as if it came from a client. Prints err too_long / busy
// when it cannot be queued. A stop is never reported as busy: CmdQueue
// raises the estop flag even when the queue is full.
bool pushFrame(const char* json, size_t len) {
  const proto::Cmd type = peekCommandType(json, len);
  if (len > proto::kMaxCmdLen) {
    printErr(type, proto::Err::TooLong, nullptr);
    return false;
  }
  if (CmdQueue::pushCommand(proto::Source::Console, 0, json, len)) return true;
  if (type == proto::Cmd::Stop) return true;
  printErr(type, proto::Err::Busy, nullptr);
  return false;
}

bool pushStr(const char* json) { return pushFrame(json, strlen(json)); }

bool pushFmt(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
bool pushFmt(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(g_cmd, sizeof(g_cmd), fmt, ap);
  va_end(ap);
  if (n <= 0 || (size_t)n >= sizeof(g_cmd)) return false;
  return pushFrame(g_cmd, (size_t)n);
}

// Integer 0..100 in s ("" or garbage -> bad_type).
void cmdRotor(const char* s) {
  char* end = nullptr;
  const long v = strtol(s, &end, 10);
  if (!*s || end == s || *end != '\0') {
    printErr(proto::Cmd::Rotor, proto::Err::BadType, "speed");
    return;
  }
  if (v < 0 || v > 100) {
    printErr(proto::Cmd::Rotor, proto::Err::OutOfRange, "speed");
    return;
  }
  pushFmt("{\"t\":\"rotor\",\"speed\":%ld}", v);
}

void cmdYaw(int8_t dir) {
  const char* name = dir < 0 ? "left" : "right";
  if (pushFmt("{\"t\":\"yaw\",\"dir\":\"%s\"}", name)) {
    g_holdDir = dir;
    g_holdSeenMoving = false;
    g_holdSetMs = millis();
  }
}

void cmdGoto(const char* s) {
  char* end = nullptr;
  const float v = strtof(s, &end);
  if (!*s || end == s || *end != '\0' || !isfinite(v)) {
    printErr(proto::Cmd::Yaw, proto::Err::BadType, "goto");
    return;
  }
  if (v < 0.0f || v >= 360.0f) {
    printErr(proto::Cmd::Yaw, proto::Err::OutOfRange, "goto");
    return;
  }
  clearHold();
  pushFmt("{\"t\":\"yaw\",\"goto\":%.1f}", (double)v);
}

void cmdBattery() {
  const int n = snprintf(g_out, sizeof(g_out),
                         "{\"t\":\"bat\",\"mv\":%lu,\"adcMv\":%lu,\"pct\":%u,\"low\":%s,\"crit\":%s,\"present\":%s}",
                         (unsigned long)Battery::mv(), (unsigned long)Battery::adcMv(), (unsigned)Battery::pct(),
                         Battery::low() ? "true" : "false", Battery::crit() ? "true" : "false",
                         Battery::present() ? "true" : "false");
  if (n > 0 && (size_t)n < sizeof(g_out)) writeLine(g_out, (size_t)n);
}

// "on" / "off" / "" (query). Returns -1 for anything else.
int onOff(const char* arg) {
  if (!strcmp(arg, "on") || !strcmp(arg, "1")) return 1;
  if (!strcmp(arg, "off") || !strcmp(arg, "0")) return 0;
  if (!*arg) return 2;
  return -1;
}

void cmdTel(const char* arg) {
  const int v = onOff(arg);
  if (v < 0) {
    note("# użycie: tel on|off");
    return;
  }
  if (v < 2) {
    g_tel = v == 1;
    g_telMs = millis() - kTelPeriodMs;  // first frame on the next tick
  }
  note("# telemetria %s", g_tel ? "włączona" : "wyłączona");
}

void cmdKa(const char* arg) {
  const int v = onOff(arg);
  if (v < 0) {
    note("# użycie: ka on|off");
    return;
  }
  if (v < 2) {
    g_ka = v == 1;
    g_kaMs = millis() - kKeepAlivePeriodMs;  // first ping on the next tick
    g_kaErrShown = false;
  }
  note("# podtrzymanie (ka) %s", g_ka ? "włączone" : "wyłączone");
}

char* trim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
  return s;
}

void handleLine(char* raw) {
  char* line = trim(raw);
  if (!*line) return;

  // JSON frame as-is. Any yaw or stop frame ends the yl / yr keepalive.
  if (line[0] == '{') {
    const size_t len = strlen(line);
    const proto::Cmd type = peekCommandType(line, len);
    if (type == proto::Cmd::Yaw || type == proto::Cmd::Stop) clearHold();
    pushFrame(line, len);
    return;
  }

  for (char* p = line; *p; p++) *p = (char)tolower((unsigned char)*p);

  // Split "word arg".
  char* arg = line;
  while (*arg && *arg != ' ' && *arg != '\t') arg++;
  if (*arg) *arg++ = '\0';
  arg = trim(arg);
  const char* w = line;

  if (!strcmp(w, "?") || !strcmp(w, "help") || !strcmp(w, "h")) {
    g_helpIdx = 0;
  } else if (!strcmp(w, "s")) {
    clearHold();
    pushStr("{\"t\":\"stop\"}");
  } else if (!strcmp(w, "rb")) {
    pushStr("{\"t\":\"rotor\",\"speed\":0,\"brake\":true}");
  } else if (w[0] == 'r') {
    cmdRotor(w[1] ? w + 1 : arg);
  } else if (!strcmp(w, "yl")) {
    cmdYaw(-1);
  } else if (!strcmp(w, "yr")) {
    cmdYaw(1);
  } else if (!strcmp(w, "ys")) {
    clearHold();
    pushStr("{\"t\":\"yaw\",\"dir\":\"stop\"}");
  } else if (w[0] == 'g') {
    cmdGoto(w[1] ? w + 1 : arg);
  } else if (!strcmp(w, "b")) {
    cmdBattery();
  } else if (!strcmp(w, "tel")) {
    cmdTel(arg);
  } else if (!strcmp(w, "ka")) {
    cmdKa(arg);
  } else {
    printErr(proto::Cmd::None, proto::Err::UnknownCmd, nullptr);
    note("# nieznana komenda, wpisz ? po pomoc");
  }
}

// ---------------------------------------------------------------------------
// Periodic work
// ---------------------------------------------------------------------------

void readInput() {
  size_t budget = kReadBudget;
  while (budget > 0 && Serial.available() > 0) {
    const int ci = Serial.read();
    if (ci < 0) break;
    budget--;
    const char c = (char)ci;

    if (c == '\n' && g_afterCr) {  // second half of \r\n
      g_afterCr = false;
      continue;
    }
    g_afterCr = c == '\r';

    if (c == '\r' || c == '\n') {
      if (g_overflow) {
        printErr(proto::Cmd::None, proto::Err::TooLong, nullptr);
        note("# linia dłuższa niż %u znaków, pominięta", (unsigned)kLineMax);
      } else {
        g_line[g_lineLen] = '\0';
        handleLine(g_line);
      }
      g_lineLen = 0;
      g_overflow = false;
      continue;
    }
    if (c == '\b' || c == 0x7f) {  // backspace from interactive terminals
      if (g_lineLen > 0 && !g_overflow) g_lineLen--;
      continue;
    }
    if ((unsigned char)c < 0x20 && c != '\t') continue;  // other control bytes
    if (g_overflow) continue;
    if (g_lineLen >= kLineMax) {
      g_overflow = true;
      continue;
    }
    g_line[g_lineLen++] = c;
  }
}

// The yl / yr keepalive ends once the yaw is seen stopped (deadman, limit,
// failsafe, another client), or if the move never started (rejected).
void observeYaw(uint32_t nowMs) {
  if (g_holdDir == 0) return;
  if (!yaw.idle()) {
    g_holdSeenMoving = true;
    return;
  }
  if (g_holdSeenMoving || (uint32_t)(nowMs - g_holdSetMs) >= kHoldStartGraceMs) clearHold();
}

void printHelp() {
  while (g_helpIdx < kHelpCount) {
    const char* s = kHelp[g_helpIdx];
    const size_t n = strlen(s);
    if (!txRoom(n)) return;  // continue on a later tick
    writeLine(s, n);
    g_helpIdx++;
  }
}

void keepAlive(uint32_t nowMs) {
  if (!g_ka || (uint32_t)(nowMs - g_kaMs) < kKeepAlivePeriodMs) return;
  g_kaMs = nowMs;
  pushStr(kPingFrame);
  if (g_holdDir == 0) return;
  if ((uint32_t)(nowMs - g_holdSetMs) >= kHoldMaxMs) {
    clearHold();
    note("# ka: podtrzymanie yaw zakończone po %lu s, wyślij yl/yr ponownie", (unsigned long)(kHoldMaxMs / 1000));
    return;
  }
  pushStr(g_holdDir < 0 ? kHoldLeftFrame : kHoldRightFrame);
}

void telemetry(uint32_t nowMs) {
  if (!g_tel || (uint32_t)(nowMs - g_telMs) < kTelPeriodMs) return;
  g_telMs = nowMs;
  const size_t n = Commands::buildTelemetry(g_out, sizeof(g_out), true, nullptr);
  if (n) writeLine(g_out, n);
}

}  // namespace

namespace SerialConsole {

void begin() {
  g_lineLen = 0;
  g_overflow = false;
  g_afterCr = false;
  g_tel = false;
  g_ka = false;
  clearHold();
  g_helpIdx = kHelpCount;
  note("# konsola gotowa, wpisz ? po pomoc");
}

void tick(uint32_t nowMs) {
  readInput();
  observeYaw(nowMs);
  printHelp();
  keepAlive(nowMs);
  telemetry(nowMs);
}

void print(const char* data, size_t len) {
  if (!data || !len) return;
  if (swallowKeepAliveReply(data, len)) return;
  writeLine(data, len);
}

void disableKeepAlive() {
  if (!g_ka) return;
  g_ka = false;
  note("# podtrzymanie (ka) wyłączone: połączył się klient WS/BLE");
}

}  // namespace SerialConsole

#else  // !ENABLE_SERIAL_CONSOLE

namespace SerialConsole {
void begin() {}
void tick(uint32_t nowMs) { (void)nowMs; }
void print(const char* data, size_t len) {
  (void)data;
  (void)len;
}
void disableKeepAlive() {}
}  // namespace SerialConsole

#endif  // ENABLE_SERIAL_CONSOLE
