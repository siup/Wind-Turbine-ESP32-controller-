#include "failsafe_logic.h"

using proto::Err;
using proto::FsReason;

namespace fs {

void FailsafeLogic::onFrame(const ClientRef& c, uint32_t nowMs) {
  if (owner_.valid() && c == owner_) ownerSeenMs_ = nowMs;
  clear(FsReason::Timeout);
  clear(FsReason::Disconnect);
}

void FailsafeLogic::onMotionCommand(const ClientRef& c, uint32_t nowMs) {
  owner_ = c;
  ownerSeenMs_ = nowMs;
  // A loss of the previous owner that update() has not seen yet no longer
  // matters: the motion now belongs to c.
  ownerLost_ = false;
}

void FailsafeLogic::releaseOwner() {
  owner_ = ClientRef{};
  ownerLost_ = false;
}

void FailsafeLogic::onClientDisconnected(const ClientRef& c, uint32_t nowMs) {
  (void)nowMs;
  if (owner_.valid() && c == owner_) ownerLost_ = true;
}

Result FailsafeLogic::update(const Inputs& in) {
  Result r;
  const uint8_t before = mask_;
  bool zero = false;

  // Level inputs: Battery mirrors the latched crit flag, Ota the upload state.
  if (in.batteryCrit) {
    set(FsReason::Battery);
  } else {
    clear(FsReason::Battery);
  }
  if (in.otaActive) {
    set(FsReason::Ota);
  } else {
    clear(FsReason::Ota);
  }

  // Runtime latches until clearRuntime(). Level triggered on purpose: if the
  // limit is still exceeded after a clear, it trips again.
  if (in.runLimitHit) {
    set(FsReason::Runtime);
    zero = true;
  }

  // The last WS / BLE client went away (edge, not state).
  if (lastClients_ > 0 && in.clients == 0) {
    set(FsReason::Disconnect);
    zero = true;
  }
  lastClients_ = in.clients;

  // The motion owner went away.
  if (ownerLost_) {
    ownerLost_ = false;
    set(FsReason::Disconnect);
    zero = true;
  }

  // The motion owner is silent. Signed age: a frame stamped slightly after
  // in.nowMs must not look like 49 days of silence.
  if (owner_.valid()) {
    const int32_t age = (int32_t)(in.nowMs - ownerSeenMs_);
    if (age > (int32_t)timeoutMs_) {
      set(FsReason::Timeout);
      zero = true;
    }
  }

  const bool hard = has(FsReason::Battery) || has(FsReason::Ota);
  if (zero || hard) {
    // Nothing moves any more, so nobody owns motion. Nothing restarts by
    // itself: the user has to start again.
    owner_ = ClientRef{};
    ownerLost_ = false;
  }
  r.zeroSetpoints = zero;
  // Repeated while active: anything that somehow moves gets stopped again.
  r.hardStop = hard;

  const uint8_t rose = (uint8_t)(mask_ & ~before);
  if (rose) {
    r.tripped = true;
    for (int i = (int)FsReason::Ota; i > (int)FsReason::None; i--) {
      if (rose & (1u << i)) {
        r.trippedReason = (FsReason)i;
        break;
      }
    }
  }
  return r;
}

void FailsafeLogic::clearRuntime() { clear(FsReason::Runtime); }

FsReason FailsafeLogic::reason() const {
  for (int i = (int)FsReason::Ota; i > (int)FsReason::None; i--) {
    if (mask_ & (1u << i)) return (FsReason)i;
  }
  return FsReason::None;
}

Err FailsafeLogic::allowMotion() const {
  if (has(FsReason::Ota)) return Err::BusyOta;
  if (has(FsReason::Battery)) return Err::LowBattery;
  return Err::None;
}

}  // namespace fs
