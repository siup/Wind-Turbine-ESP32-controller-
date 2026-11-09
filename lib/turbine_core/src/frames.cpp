#include "frames.h"

#include <string.h>

#include <cmath>

using proto::FsReason;
using proto::YawDir;

namespace {

// Minimal streaming JSON writer into a fixed buffer. No allocation, no
// printf. Output is pure ASCII: control characters are escaped and bytes
// >= 0x80 become '?', so a text WebSocket frame is always valid UTF-8.
class JsonOut {
 public:
  JsonOut(char* out, size_t cap) : out_(out), cap_(cap) {
    if (!out_ || cap_ == 0) ok_ = false;
  }

  // Length written (excluding NUL), or 0 if the frame did not fit.
  size_t finish() {
    if (!ok_) {
      if (out_ && cap_) out_[0] = '\0';
      return 0;
    }
    out_[len_] = '\0';
    return len_;
  }

  void beginObj() { open('{'); }
  void endObj() { close('}'); }
  void beginArr() { open('['); }
  void endArr() { close(']'); }

  // Next object member: comma if needed, then "key":
  void key(const char* k) {
    sep();
    put('"');
    putRaw(k, strlen(k));
    put('"');
    put(':');
  }
  // Next array element (comma if needed).
  void elem() { sep(); }

  void u32(uint32_t v) {
    char tmp[10];
    int n = 0;
    do {
      tmp[n++] = (char)('0' + v % 10);
      v /= 10;
    } while (v);
    while (n) put(tmp[--n]);
  }
  void i32(int32_t v) {
    if (v < 0) {
      put('-');
      u32((uint32_t)(-(int64_t)v));
    } else {
      u32((uint32_t)v);
    }
  }
  void boolean(bool b) { b ? putRaw("true", 4) : putRaw("false", 5); }
  void null() { putRaw("null", 4); }

  // Fixed-point number with up to `decimals` (1..3) digits after the point,
  // trailing zeros trimmed but at least one kept: 1.0, 0.987, 1.25.
  void fixed(double v, int decimals) {
    if (!std::isfinite(v)) {
      null();
      return;
    }
    static const int64_t kScale[] = {1, 10, 100, 1000};
    if (decimals < 1) decimals = 1;
    if (decimals > 3) decimals = 3;
    int64_t scale = kScale[decimals];
    double scaled = std::round(v * (double)scale);
    if (scaled > 4.0e18 || scaled < -4.0e18) {
      null();
      return;
    }
    int64_t m = (int64_t)scaled;
    if (m < 0) {
      put('-');
      m = -m;
    }
    int64_t ip = m / scale;
    int64_t fp = m % scale;
    if (ip > 0xFFFFFFFFll) {
      null();
      return;
    }
    u32((uint32_t)ip);
    put('.');
    char digits[3];
    for (int i = decimals - 1; i >= 0; i--) {
      digits[i] = (char)('0' + fp % 10);
      fp /= 10;
    }
    int keep = decimals;
    while (keep > 1 && digits[keep - 1] == '0') keep--;
    putRaw(digits, (size_t)keep);
  }

  // Quoted, escaped string; nullptr becomes null. maxLen bounds the read
  // (for fixed char arrays that may lack a NUL).
  void str(const char* s, size_t maxLen = (size_t)-1) {
    if (!s) {
      null();
      return;
    }
    static const char kHex[] = "0123456789abcdef";
    put('"');
    for (size_t i = 0; i < maxLen && s[i]; i++) {
      unsigned char c = (unsigned char)s[i];
      if (c == '"' || c == '\\') {
        put('\\');
        put((char)c);
      } else if (c < 0x20 || c == 0x7F) {
        putRaw("\\u00", 4);
        put(kHex[c >> 4]);
        put(kHex[c & 15]);
      } else if (c >= 0x80) {
        put('?');
      } else {
        put((char)c);
      }
    }
    put('"');
  }

  // Convenience members.
  void kvU32(const char* k, uint32_t v) {
    key(k);
    u32(v);
  }
  void kvI32(const char* k, int32_t v) {
    key(k);
    i32(v);
  }
  void kvBool(const char* k, bool v) {
    key(k);
    boolean(v);
  }
  void kvStr(const char* k, const char* v, size_t maxLen = (size_t)-1) {
    key(k);
    str(v, maxLen);
  }
  void kvNull(const char* k) {
    key(k);
    null();
  }

 private:
  void put(char c) {
    if (!ok_) return;
    if (len_ + 1 >= cap_) {  // keep room for the NUL
      ok_ = false;
      return;
    }
    out_[len_++] = c;
  }
  void putRaw(const char* s, size_t n) {
    for (size_t i = 0; i < n; i++) put(s[i]);
  }
  void sep() {
    if (depth_ == 0) return;
    uint32_t bit = 1u << (depth_ - 1);
    if (hasItem_ & bit) put(',');
    hasItem_ |= bit;
  }
  void open(char c) {
    put(c);
    if (depth_ < 32) {
      depth_++;
      hasItem_ &= ~(1u << (depth_ - 1));
    } else {
      ok_ = false;
    }
  }
  void close(char c) {
    if (depth_ > 0) depth_--;
    put(c);
  }

  char* out_;
  size_t cap_;
  size_t len_ = 0;
  bool ok_ = true;
  uint8_t depth_ = 0;
  uint32_t hasItem_ = 0;  // bit d set = level d already has a member
};

void writeHeader(JsonOut& w, const char* type) {
  w.beginObj();
  w.kvStr("t", type);
}

void writeTail(JsonOut& w, bool hasId, uint32_t id) {
  if (hasId) w.kvU32("id", id);
  w.endObj();
}

const char* movingName(YawDir d) {
  switch (d) {
    case YawDir::Left: return "left";
    case YawDir::Right: return "right";
    case YawDir::Stop: break;
  }
  return nullptr;
}

const char* twistName(int8_t t) {
  if (t < 0) return "L";
  if (t > 0) return "R";
  return nullptr;
}

void writeLogItem(JsonOut& w, const LogItem& it) {
  w.kvU32("ms", it.ms);
  char lvl[2] = {it.lvl ? it.lvl : 'i', '\0'};
  w.kvStr("lvl", lvl);
  w.kvStr("code", it.code, sizeof(it.code));
  w.kvStr("msg", it.msg, sizeof(it.msg));
}

// FNV-1a, 32 bit.
struct Fnv {
  uint32_t h = 2166136261u;
  void byte(uint8_t b) {
    h ^= b;
    h *= 16777619u;
  }
  void u32(uint32_t v) {
    for (int i = 0; i < 4; i++) byte((uint8_t)(v >> (8 * i)));
  }
  void i32(int32_t v) { u32((uint32_t)v); }
  void str(const char* s) {
    if (s) {
      while (*s) byte((uint8_t)*s++);
    }
    byte(0);
  }
};

}  // namespace

size_t buildState(const TelemetrySnapshot& s, bool brief, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "state");
  w.kvI32("v", proto::kVersion);

  w.key("rotor");
  w.beginObj();
  w.kvU32("target", s.rotorTarget);
  w.kvU32("actual", s.rotorActual);
  if (!brief) w.kvU32("run", s.rotorRunS);
  w.endObj();

  w.key("yaw");
  w.beginObj();
  w.kvStr("moving", movingName(s.yawMoving));
  if (!brief) w.kvU32("speed", s.yawSpeed);
  if (s.hasAngle) {
    w.kvI32("angle", s.angle);
  } else {
    w.kvNull("angle");
  }
  if (!brief) {
    w.kvBool("limitL", s.limitL);
    w.kvBool("limitR", s.limitR);
    w.kvStr("twist", twistName(s.twist));
    w.kvI32("bal", s.bal);
    w.kvU32("balMax", s.balMax);
    if (s.hasPos) {
      w.kvI32("pos", s.pos);
    } else {
      w.kvNull("pos");
    }
    w.kvBool("est", s.posEst);
    if (s.hasPosMax) {
      w.kvI32("posMax", s.posMax);
    } else {
      w.kvNull("posMax");
    }
    if (s.gotoActive) {
      w.kvI32("goto", s.gotoTarget);
    } else {
      w.kvNull("goto");
    }
    w.kvStr("sensor", s.sensor ? s.sensor : "none");
    w.kvBool("sensorOk", s.sensorOk);
    w.kvBool("cal", s.calActive);
  }
  w.endObj();

  w.key("bat");
  w.beginObj();
  if (!brief) w.kvU32("mv", s.batMv);
  w.kvU32("pct", s.batPct);
  if (!brief) w.kvBool("low", s.batLow);
  w.kvBool("crit", s.batCrit);
  if (!brief) w.kvBool("present", s.batPresent);
  w.endObj();

  if (!brief) {
    w.key("link");
    w.beginObj();
    if (s.hasRssi) {
      w.kvI32("rssi", s.rssi);
    } else {
      w.kvNull("rssi");
    }
    w.kvU32("clients", (uint32_t)s.ws + (uint32_t)s.ble);
    w.kvU32("ws", s.ws);
    w.kvU32("ble", s.ble);
    w.kvU32("sta", s.sta);
    w.endObj();
  }

  w.key("fs");
  w.beginObj();
  w.kvBool("active", s.fsActive);
  w.kvStr("reason", proto::fsReasonName(s.fsReason));
  w.endObj();

  if (!brief) {
    w.kvBool("ota", s.ota);
    w.kvU32("cfgRev", s.cfgRev);
    w.kvU32("up", s.upMs);
  }
  w.endObj();
  return w.finish();
}

uint32_t stateSignature(const TelemetrySnapshot& s) {
  Fnv h;
  h.u32(s.rotorTarget);
  h.u32(s.rotorActual);

  h.i32((int32_t)s.yawMoving);
  h.u32(s.yawSpeed);
  // A stationary AS5600 may dither by a degree; while the yaw is idle only
  // a move of 2 degrees or more counts as a change (the periodic frame still
  // carries the exact value). While moving every degree is sent at once.
  const bool yawIdle = s.yawMoving == YawDir::Stop && !s.gotoActive;
  h.u32(s.hasAngle);
  if (s.hasAngle) h.i32(yawIdle ? s.angle / 2 : s.angle);
  h.u32(s.limitL);
  h.u32(s.limitR);
  h.i32(s.twist);
  h.i32(s.bal);
  h.u32(s.balMax);
  h.u32(s.hasPos);
  if (s.hasPos) h.i32(yawIdle ? s.pos / 2 : s.pos);
  h.u32(s.posEst);
  h.u32(s.hasPosMax);
  if (s.hasPosMax) h.i32(s.posMax);
  h.u32(s.gotoActive);
  if (s.gotoActive) h.i32(s.gotoTarget);
  h.str(s.sensor);
  h.u32(s.sensorOk);
  h.u32(s.calActive);

  h.u32(s.batPct);
  h.u32(s.batLow);
  h.u32(s.batCrit);
  h.u32(s.batPresent);

  h.u32(s.ws);
  h.u32(s.ble);
  h.u32(s.sta);

  h.u32(s.fsActive);
  h.u32((uint32_t)s.fsReason);
  h.u32(s.ota);
  h.u32(s.cfgRev);
  return h.h;
}

size_t buildAck(proto::Cmd of, bool hasId, uint32_t id, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "ack");
  w.kvStr("of", proto::cmdName(of));
  writeTail(w, hasId, id);
  return w.finish();
}

size_t buildErr(proto::Cmd of, proto::Err e, const char* field, const char* msg, bool hasId,
                uint32_t id, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "err");
  w.kvStr("of", proto::cmdName(of));
  w.kvStr("code", proto::errCode(e));
  if (field) w.kvStr("field", field);
  w.kvStr("msg", msg ? msg : proto::errMessage(e));
  writeTail(w, hasId, id);
  return w.finish();
}

size_t buildPong(bool hasId, uint32_t id, uint32_t upMs, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "pong");
  if (hasId) w.kvU32("id", id);
  w.kvU32("up", upMs);
  w.endObj();
  return w.finish();
}

size_t buildCfg(const Settings& s, uint32_t rev, bool rebootPending, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "cfg");
  w.kvU32("rev", rev);
  w.kvBool("reboot", rebootPending);
  const FieldDesc* pass = settingsFindField("apPass");
  w.kvBool("apPassSet", pass && settingsGetStr(s, *pass)[0] != '\0');
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    if (f.flags & kFieldSecret) continue;
    w.key(f.key);
    switch (f.type) {
      case FieldType::Str:
        w.str(settingsGetStr(s, f), f.size);
        break;
      case FieldType::Bool:
        w.boolean(settingsGetNumber(s, f) != 0.0);
        break;
      case FieldType::Float:
        w.fixed(settingsGetNumber(s, f), 3);
        break;
      case FieldType::U8:
      case FieldType::U16:
      case FieldType::U32:
        w.u32((uint32_t)settingsGetNumber(s, f));
        break;
    }
  }
  w.endObj();
  return w.finish();
}

size_t buildInfo(const DeviceInfo& d, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "info");
  w.kvI32("v", proto::kVersion);
  w.kvStr("fw", d.fw ? d.fw : "");
  w.kvStr("build", d.build ? d.build : "");
  w.kvStr("name", d.name ? d.name : "");
  w.kvStr("chip", d.chip ? d.chip : "");
  w.kvStr("mac", d.mac ? d.mac : "");
  w.kvStr("sensor", d.sensor ? d.sensor : "none");
  w.kvBool("wifi", d.wifi);
  w.kvBool("ble", d.ble);
  w.kvBool("ota", d.ota);
  w.kvU32("heap", d.heap);
  w.kvU32("heapMin", d.heapMin);
  w.kvStr("reset", d.resetReason ? d.resetReason : "");
  w.kvU32("up", d.upMs);
  w.endObj();
  return w.finish();
}

size_t buildLog(const LogItem* items, size_t n, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "log");
  w.key("items");
  w.beginArr();
  for (size_t i = 0; items && i < n; i++) {
    w.elem();
    w.beginObj();
    writeLogItem(w, items[i]);
    w.endObj();
  }
  w.endArr();
  w.endObj();
  return w.finish();
}

size_t buildEvt(const LogItem& item, char* out, size_t cap) {
  JsonOut w(out, cap);
  writeHeader(w, "evt");
  writeLogItem(w, item);
  w.endObj();
  return w.finish();
}
