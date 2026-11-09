#include "cmd_parser.h"

#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>

#include <cmath>
#include <cstddef>

using proto::Cmd;
using proto::Err;
using proto::YawAction;
using proto::YawDir;

namespace {

// ---------------------------------------------------------------------------
// Memory for the JSON document: a bump arena on the caller's stack, so a
// command parse normally does not touch the heap at all. Frames are capped at
// kMaxCmdLen, which bounds what the document can need; if the arena still
// runs out the allocator falls back to malloc instead of failing.
// ---------------------------------------------------------------------------
class ArenaAllocator : public ArduinoJson::Allocator {
 public:
  ArenaAllocator(unsigned char* buf, size_t cap) : buf_(buf), cap_(cap) {}

  void* allocate(size_t size) override {
    void* p = carve(size);
    return p ? p : malloc(size);
  }

  void deallocate(void* ptr) override {
    if (!ptr) return;
    if (!owns(ptr)) {
      free(ptr);
      return;
    }
    size_t off = blockOffset(ptr);
    if (off == last_) {  // only the newest block can be given back
      used_ = last_;
      last_ = kNone;
    }
  }

  void* reallocate(void* ptr, size_t size) override {
    if (!ptr) return allocate(size);
    if (!owns(ptr)) return realloc(ptr, size);
    size_t off = blockOffset(ptr);
    size_t old = sizeAt(off);
    if (off == last_) {
      size_t need = kHdr + roundUp(size);
      if (need >= size && off + need <= cap_) {  // grow or shrink in place
        setSize(off, size);
        used_ = off + need;
        return ptr;
      }
    } else if (size <= old) {
      return ptr;  // older block: shrink in place, tail unused until the parse ends
    }
    void* q = allocate(size);
    if (!q) return nullptr;
    memcpy(q, ptr, old < size ? old : size);
    deallocate(ptr);
    return q;
  }

 private:
  static constexpr size_t kAlign = alignof(std::max_align_t);
  static constexpr size_t kHdr = kAlign;  // block size stored in front of each block
  static constexpr size_t kNone = (size_t)-1;

  static size_t roundUp(size_t n) { return (n + kAlign - 1) & ~(kAlign - 1); }

  bool owns(const void* p) const {
    uintptr_t a = (uintptr_t)p, b = (uintptr_t)buf_;
    return a >= b && a < b + cap_;
  }
  size_t blockOffset(const void* p) const {
    return (size_t)((const unsigned char*)p - buf_) - kHdr;
  }
  size_t sizeAt(size_t off) const {
    size_t n;
    memcpy(&n, buf_ + off, sizeof(n));
    return n;
  }
  void setSize(size_t off, size_t n) { memcpy(buf_ + off, &n, sizeof(n)); }

  void* carve(size_t size) {
    size_t need = kHdr + roundUp(size);
    if (need < size || used_ > cap_ || need > cap_ - used_) return nullptr;
    size_t off = used_;
    setSize(off, size);
    last_ = off;
    used_ += need;
    return buf_ + off + kHdr;
  }

  unsigned char* buf_;
  size_t cap_;
  size_t used_ = 0;
  size_t last_ = kNone;
};

static_assert(sizeof(size_t) <= alignof(std::max_align_t), "arena header must hold a size_t");

// One variant pool of ArduinoJson (128 slots of 8 bytes on the ESP32, 256 x
// 16 on 64-bit hosts) plus about 15 key strings. Every rotor / yaw / ping /
// small cfg frame fits; only a cfg patch with many keys spills to the heap.
// Stack cost on the ESP32: about 1.6 kB while a frame is parsed.
constexpr size_t kArenaBytes = sizeof(void*) <= 4 ? 1536 : 5120;

struct NamedCmd {
  const char* name;
  Cmd cmd;
};

const NamedCmd kCmds[] = {
    {"rotor", Cmd::Rotor}, {"yaw", Cmd::Yaw}, {"stop", Cmd::Stop},     {"ping", Cmd::Ping},
    {"cfg", Cmd::Cfg},     {"info", Cmd::Info}, {"log", Cmd::Log}, {"reboot", Cmd::Reboot},
};

Cmd cmdFromName(const char* s, size_t n) {
  for (const NamedCmd& c : kCmds) {
    if (strlen(c.name) == n && memcmp(c.name, s, n) == 0) return c.cmd;
  }
  return Cmd::None;
}

bool isWs(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

ParseResult fail(Err e, const char* field) {
  ParseResult r;
  r.err = e;
  r.field = field;
  return r;
}

bool strEq(JsonVariantConst v, const char* s) {
  const char* p = v.as<const char*>();
  return p && strcmp(p, s) == 0;
}

// Rounds a JSON number to an integer percentage within [lo, hi].
bool roundedPct(JsonVariantConst v, int lo, int hi, uint8_t& out) {
  double d = v.as<double>();
  if (!std::isfinite(d)) return false;
  double r = std::round(d);
  if (r < lo || r > hi) return false;
  out = (uint8_t)r;
  return true;
}

// Type and range check of one cfg value, applied to `work`.
ParseResult applyField(Settings& work, const FieldDesc& f, JsonVariantConst v) {
  switch (f.type) {
    case FieldType::Str:
      if (!v.is<const char*>()) return fail(Err::BadType, f.key);
      if (!settingsSetStr(work, f, v.as<const char*>())) return fail(Err::OutOfRange, f.key);
      break;
    case FieldType::Bool:
      if (!v.is<bool>()) return fail(Err::BadType, f.key);
      settingsSetNumber(work, f, v.as<bool>() ? 1.0 : 0.0);
      break;
    case FieldType::U8:
    case FieldType::U16:
    case FieldType::U32:
    case FieldType::Float:
      if (!v.is<double>()) return fail(Err::BadType, f.key);
      if (!settingsSetNumber(work, f, v.as<double>())) return fail(Err::OutOfRange, f.key);
      break;
  }
  return ParseResult{};
}

ParseResult parseRotor(JsonObjectConst obj, Command& out) {
  JsonVariantConst sp = obj["speed"];
  if (!sp.is<double>()) return fail(Err::BadType, "speed");
  if (!roundedPct(sp, 0, 100, out.speed)) return fail(Err::OutOfRange, "speed");
  JsonVariantConst br = obj["brake"];
  if (!br.isNull()) {
    if (!br.is<bool>()) return fail(Err::BadType, "brake");
    out.brake = br.as<bool>();
  }
  return ParseResult{};
}

ParseResult parseYaw(JsonObjectConst obj, Command& out) {
  static const char* const kKeys[] = {"dir", "hold", "goto", "zero", "cal"};
  int which = -1;
  for (int i = 0; i < 5; i++) {
    if (obj[kKeys[i]].isNull()) continue;
    if (which >= 0) return fail(Err::BadType, kKeys[i]);  // exactly one allowed
    which = i;
  }
  if (which < 0) return fail(Err::BadType, "dir");
  JsonVariantConst v = obj[kKeys[which]];

  switch (which) {
    case 0: {  // dir
      if (!v.is<const char*>()) return fail(Err::BadType, "dir");
      out.yawAction = YawAction::Move;
      if (strEq(v, "left")) {
        out.dir = YawDir::Left;
      } else if (strEq(v, "right")) {
        out.dir = YawDir::Right;
      } else if (strEq(v, "stop")) {
        out.dir = YawDir::Stop;
        return ParseResult{};  // a stop never fails on an optional field
      } else {
        return fail(Err::OutOfRange, "dir");
      }
      JsonVariantConst sp = obj["speed"];
      if (!sp.isNull()) {
        if (!sp.is<double>()) return fail(Err::BadType, "speed");
        if (!roundedPct(sp, 20, 100, out.yawSpeed)) return fail(Err::OutOfRange, "speed");
      }
      return ParseResult{};
    }
    case 1:  // hold
      if (!v.is<const char*>()) return fail(Err::BadType, "hold");
      out.yawAction = YawAction::Hold;
      if (strEq(v, "left")) {
        out.dir = YawDir::Left;
      } else if (strEq(v, "right")) {
        out.dir = YawDir::Right;
      } else {
        return fail(Err::OutOfRange, "hold");
      }
      return ParseResult{};
    case 2: {  // goto
      if (!v.is<double>()) return fail(Err::BadType, "goto");
      double d = v.as<double>();
      if (!std::isfinite(d) || d < 0.0 || d >= 360.0) return fail(Err::OutOfRange, "goto");
      float f = (float)d;
      if (f >= 360.0f) f = 0.0f;  // 359.99999999 rounds up to 360 in float: same heading
      out.yawAction = YawAction::Goto;
      out.gotoDeg = f;
      return ParseResult{};
    }
    case 3:  // zero
      if (!v.is<bool>()) return fail(Err::BadType, "zero");
      if (!v.as<bool>()) return fail(Err::OutOfRange, "zero");
      out.yawAction = YawAction::Zero;
      return ParseResult{};
    default:  // cal
      if (!v.is<const char*>()) return fail(Err::BadType, "cal");
      if (strEq(v, "start")) {
        out.yawAction = YawAction::CalStart;
      } else if (strEq(v, "stop")) {
        out.yawAction = YawAction::CalStop;
      } else if (strEq(v, "cancel")) {
        out.yawAction = YawAction::CalCancel;
      } else {
        return fail(Err::OutOfRange, "cal");
      }
      return ParseResult{};
  }
}

ParseResult parseCfg(JsonObjectConst obj, Command& out, const Settings* cfgBase,
                     Settings* cfgOut) {
  JsonVariantConst get = obj["get"];
  JsonVariantConst reset = obj["reset"];
  if (!get.isNull() && !get.is<bool>()) return fail(Err::BadType, "get");
  if (!reset.isNull() && !reset.is<bool>()) return fail(Err::BadType, "reset");
  if (reset.as<bool>()) {
    out.cfgReset = true;
    return ParseResult{};
  }
  if (get.as<bool>()) {
    out.cfgGet = true;
    return ParseResult{};
  }

  // Patch a private copy; cfgBase and cfgOut may point to the same struct.
  Settings work;
  if (cfgBase) {
    work = *cfgBase;
  } else {
    settingsDefaults(work);
  }

  // Battery preset first, so explicit thresholds in the same patch win.
  JsonVariantConst bt = obj["batType"];
  if (!bt.isNull()) {
    const FieldDesc* f = settingsFindField("batType");
    ParseResult r = applyField(work, *f, bt);
    if (r.err != Err::None) return r;
    if (work.batType != 2) applyBatteryPreset(work, work.batType);
  }

  uint64_t mask = 0;
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    const FieldDesc& f = kSettingsFields[i];
    JsonVariantConst v = obj[f.key];
    if (v.isNull()) continue;
    ParseResult r = applyField(work, f, v);
    if (r.err != Err::None) return r;
    mask |= (uint64_t)1 << i;
  }

  if (mask == 0) {  // {"t":"cfg"} or only unknown keys: a read
    out.cfgGet = true;
    return ParseResult{};
  }

  if (cfgBase) {
    int bad = settingsValidate(work);
    if (bad >= 0) return fail(Err::OutOfRange, kSettingsFields[bad].key);
    if (cfgOut) *cfgOut = work;
  }
  out.cfgMask = mask;
  return ParseResult{};
}

}  // namespace

proto::Cmd peekCommandType(const char* json, size_t len) {
  if (!json) return Cmd::None;
  size_t i = 0;
  auto atEnd = [&]() { return i >= len || json[i] == '\0'; };

  while (!atEnd() && isWs(json[i])) i++;
  if (atEnd() || json[i] != '{') return Cmd::None;
  i++;

  int depth = 1;
  bool expectKey = true;   // next string at depth 1 is a key
  Cmd found = Cmd::None;   // last top-level "t" wins, as in ArduinoJson
  while (!atEnd()) {
    char c = json[i];
    if (c == '"') {
      size_t start = ++i;
      bool escaped = false;
      while (!atEnd() && json[i] != '"') {
        if (json[i] == '\\') {
          escaped = true;
          i++;
          if (atEnd()) break;
        }
        i++;
      }
      if (atEnd()) return found;  // truncated string: keep what we know
      size_t n = i - start;
      i++;  // closing quote
      if (depth != 1 || !expectKey) continue;
      expectKey = false;
      if (escaped) return Cmd::None;  // an escaped key might spell "t"
      if (n != 1 || json[start] != 't') continue;

      // "t" key: its value must be a plain string
      while (!atEnd() && isWs(json[i])) i++;
      if (atEnd() || json[i] != ':') return Cmd::None;
      i++;
      while (!atEnd() && isWs(json[i])) i++;
      if (atEnd()) return Cmd::None;
      if (json[i] != '"') {
        found = Cmd::None;  // not a string: the parser rejects it
        continue;
      }
      size_t vs = ++i;
      while (!atEnd() && json[i] != '"' && json[i] != '\\') i++;
      if (atEnd() || json[i] == '\\') return Cmd::None;
      found = cmdFromName(json + vs, i - vs);
      i++;
      continue;
    }
    if (c == '{' || c == '[') {
      depth++;
    } else if (c == '}' || c == ']') {
      if (--depth == 0) break;
    } else if (c == ',' && depth == 1) {
      expectKey = true;
    }
    i++;
  }
  return found;
}

ParseResult parseCommand(const char* json, size_t len, Command& out, const Settings* cfgBase,
                         Settings* cfgOut) {
  out = Command{};
  if (!json) return fail(Err::BadJson, nullptr);
  if (len > proto::kMaxCmdLen) {
    out.type = peekCommandType(json, len);  // best effort for the "of" field
    return fail(Err::TooLong, nullptr);
  }

  alignas(std::max_align_t) unsigned char arenaBuf[kArenaBytes];
  ArenaAllocator arena(arenaBuf, sizeof(arenaBuf));
  JsonDocument doc(&arena);
  DeserializationError de = deserializeJson(doc, json, len);
  if (de || !doc.is<JsonObjectConst>()) return fail(Err::BadJson, nullptr);
  JsonObjectConst obj = doc.as<JsonObjectConst>();

  // "id" first so that every later error can still be paired by the client.
  // Null counts as absent everywhere (optional fields).
  bool idBad = false;
  JsonVariantConst idv = obj["id"];
  if (!idv.isNull()) {
    if (idv.is<uint32_t>()) {
      out.hasId = true;
      out.id = idv.as<uint32_t>();
    } else {
      idBad = true;
    }
  }

  JsonVariantConst tv = obj["t"];
  if (!tv.is<const char*>()) return fail(Err::BadType, "t");
  const char* t = tv.as<const char*>();
  out.type = cmdFromName(t, strlen(t));
  if (out.type == Cmd::None) return fail(Err::UnknownCmd, nullptr);
  if (idBad) return fail(Err::BadType, "id");

  switch (out.type) {
    case Cmd::Rotor:
      return parseRotor(obj, out);
    case Cmd::Yaw:
      return parseYaw(obj, out);
    case Cmd::Cfg:
      return parseCfg(obj, out, cfgBase, cfgOut);
    case Cmd::Stop:
    case Cmd::Ping:
    case Cmd::Info:
    case Cmd::Log:
    case Cmd::Reboot:
    case Cmd::None:
      break;
  }
  return ParseResult{};
}
