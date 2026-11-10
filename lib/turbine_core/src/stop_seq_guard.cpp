#include "stop_seq_guard.h"

using proto::Cmd;
using proto::YawAction;
using proto::YawDir;

namespace {

enum class Kind : uint8_t { Other, Stop, Motion };

struct Kinds {
  Kind rotor = Kind::Other;
  Kind yaw = Kind::Other;
};

Kinds classify(const Command& c) {
  Kinds k;
  switch (c.type) {
    case Cmd::Stop:
      k.rotor = Kind::Stop;
      k.yaw = Kind::Stop;
      break;
    case Cmd::Rotor:
      k.rotor = c.speed > 0 ? Kind::Motion : Kind::Stop;
      break;
    case Cmd::Yaw:
      if (c.yawAction == YawAction::Move) {
        k.yaw = c.dir == YawDir::Stop ? Kind::Stop : Kind::Motion;
      } else if (c.yawAction == YawAction::Goto) {
        k.yaw = Kind::Motion;
      }
      break;
    default:
      break;
  }
  return k;
}

// a is older than b (wrap-safe for differences below 2^31).
bool older(uint32_t a, uint32_t b) { return (int32_t)(a - b) < 0; }

}  // namespace

void StopSeqGuard::reset() {
  for (Slot& s : slots_) s = Slot{};
}

StopSeqGuard::Slot* StopSeqGuard::find(uint32_t client) {
  for (Slot& s : slots_) {
    if (s.used && s.client == client) return &s;
  }
  return nullptr;
}

StopSeqGuard::Slot* StopSeqGuard::take(uint32_t client, uint32_t nowMs) {
  Slot* s = find(client);
  if (!s) {
    // A free slot, else the least recently used one.
    s = &slots_[0];
    for (Slot& c : slots_) {
      if (!c.used) {
        s = &c;
        break;
      }
      if ((int32_t)(c.touchMs - s->touchMs) < 0) s = &c;
    }
    *s = Slot{};
    s->used = true;
    s->client = client;
  }
  s->touchMs = nowMs;
  return s;
}

bool StopSeqGuard::accept(uint32_t client, const Command& c, uint32_t nowMs) {
  if (!c.hasSeq) return true;
  const Kinds k = classify(c);
  if (k.rotor == Kind::Other && k.yaw == Kind::Other) return true;

  if (k.rotor == Kind::Stop || k.yaw == Kind::Stop) {
    Slot* s = take(client, nowMs);
    auto mark = [&](Axis& a) {
      const bool expired = (uint32_t)(nowMs - a.atMs) >= kWindowMs;
      // Keep the newest stop: a late stop must not reopen the window for
      // motion frames sent between it and the newer one.
      if (!a.has || expired || older(a.seq, c.seq)) a.seq = c.seq;
      a.has = true;
      a.atMs = nowMs;
    };
    if (k.rotor == Kind::Stop) mark(s->rotor);
    if (k.yaw == Kind::Stop) mark(s->yaw);
    return true;
  }

  // Motion on one axis.
  Slot* s = find(client);
  if (!s) return true;
  const Axis& a = k.rotor == Kind::Motion ? s->rotor : s->yaw;
  if (!a.has || (uint32_t)(nowMs - a.atMs) >= kWindowMs) return true;
  return !older(c.seq, a.seq);
}
