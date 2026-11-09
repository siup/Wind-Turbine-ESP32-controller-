#include "yaw_logic.h"

#include <cmath>

namespace {

// Keeps balance() inside int32 even after absurdly long runs.
constexpr int64_t kAccLimit = (int64_t)2000000000 * 1000;

int64_t clampAcc(int64_t v) {
  if (v > kAccLimit) return kAccLimit;
  if (v < -kAccLimit) return -kAccLimit;
  return v;
}

}  // namespace

// ---------------------------------------------------------------------------
// YawBalance
// ---------------------------------------------------------------------------

void YawBalance::reset() {
  accMicro_ = 0;
  refL_ = refR_ = false;
  refLMicro_ = refRMicro_ = 0;
}

void YawBalance::restore(int32_t balance) {
  reset();
  accMicro_ = (int64_t)balance * 1000;
}

int32_t YawBalance::balance() const {
  // Rounded half away from zero, like Ramp.
  int64_t b = accMicro_ >= 0 ? (accMicro_ + 500) / 1000 : (accMicro_ - 500) / 1000;
  return (int32_t)b;
}

void YawBalance::integrate(int8_t dirSign, uint8_t speedPct, uint32_t dtMs) {
  if (dirSign == 0 || speedPct == 0 || dtMs == 0) return;
  if (speedPct > 100) speedPct = 100;
  // ms * pct / 100 ms-at-100 %, in thousandths: ms * pct * 10. Exact.
  int64_t d = (int64_t)dtMs * speedPct * 10;
  accMicro_ = clampAcc(accMicro_ + (dirSign > 0 ? d : -d));
}

bool YawBalance::blocked(int8_t dirSign, uint32_t limit) const {
  if (limit == 0 || dirSign == 0) return false;
  int64_t b = balance();
  return dirSign > 0 ? b >= (int64_t)limit : b <= -(int64_t)limit;
}

void YawBalance::onEndstop(int8_t side) {
  if (side < 0) {
    if (refL_) {
      accMicro_ = refLMicro_;
    } else {
      refL_ = true;
      refLMicro_ = accMicro_;
    }
  } else if (side > 0) {
    if (refR_) {
      accMicro_ = refRMicro_;
    } else {
      refR_ = true;
      refRMicro_ = accMicro_;
    }
  }
}

bool YawBalance::hasReference(int8_t side) const {
  if (side < 0) return refL_;
  if (side > 0) return refR_;
  return false;
}

bool YawBalance::estimateDeg(uint32_t msPer360, float& outDeg) const {
  if (msPer360 == 0) return false;
  outDeg = (float)((double)accMicro_ / 1000.0 / (double)msPer360 * 360.0);
  return true;
}

// ---------------------------------------------------------------------------
// EndstopDebouncer
// ---------------------------------------------------------------------------

void EndstopDebouncer::reset(bool rawActive, uint32_t nowMs) {
  stable_ = rawActive;
  last_ = rawActive;
  rose_ = false;
  changedAt_ = nowMs;
}

bool EndstopDebouncer::update(bool rawActive, uint32_t nowMs, uint32_t debounceMs) {
  rose_ = false;
  if (rawActive != last_) {
    last_ = rawActive;
    changedAt_ = nowMs;
  }
  if (last_ != stable_ && (uint32_t)(nowMs - changedAt_) >= debounceMs) {
    stable_ = last_;
    rose_ = stable_;
  }
  return stable_;
}

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------

float median3(float a, float b, float c) {
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  // a <= b
  if (c <= a) return a;
  if (c >= b) return b;
  return c;
}

float wrap360(float deg) {
  if (!std::isfinite(deg)) return 0.0f;
  float r = std::fmod(deg, 360.0f);
  if (r < 0.0f) r += 360.0f;
  if (r >= 360.0f) r = 0.0f;  // -1e-6 + 360 rounds to 360 in float
  return r;
}

float shortestDelta(float fromAbs, float toAbs) {
  float d = wrap360(toAbs - fromAbs);  // [0, 360)
  if (d > 180.0f) d -= 360.0f;         // (-180, 180]
  return d;
}

// ---------------------------------------------------------------------------
// AngleUnwrapper
// ---------------------------------------------------------------------------
// The continuous angle is always rebuilt as turns * 360 + reading, so float
// rounding never accumulates over many updates.

void AngleUnwrapper::reset(float hintCont) {
  init_ = false;
  hint_ = std::isfinite(hintCont) ? hintCont : 0.0f;
  lastAbs_ = wrap360(hint_);
  cont_ = hint_;
}

float AngleUnwrapper::update(float absDeg) {
  const float a = wrap360(absDeg);
  float turns;
  if (!init_) {
    turns = std::round((hint_ - a) / 360.0f);  // nearest to the hint
    init_ = true;
  } else {
    turns = std::round((cont_ - lastAbs_) / 360.0f);
    const float step = a - lastAbs_;
    if (step > 180.0f) turns -= 1.0f;   // crossed 0 going left (e.g. 2 -> 358)
    if (step < -180.0f) turns += 1.0f;  // crossed 0 going right (e.g. 358 -> 2)
  }
  lastAbs_ = a;
  cont_ = turns * 360.0f + a;
  return cont_;
}

// ---------------------------------------------------------------------------
// Goto planner
// ---------------------------------------------------------------------------

GotoStep planGoto(float currentCont, float targetAbs, uint8_t cruisePct, float maxTwistDeg,
                  const GotoParams& p) {
  GotoStep st;
  if (!std::isfinite(currentCont) || !std::isfinite(targetAbs)) {
    st.impossible = true;
    return st;
  }
  const float shortWay = shortestDelta(wrap360(currentCont), targetAbs);
  if (std::fabs(shortWay) <= p.windowDeg) {
    st.done = true;
    return st;
  }

  float delta = shortWay;
  if (maxTwistDeg > 0.0f) {
    const float longWay = shortWay > 0.0f ? shortWay - 360.0f : shortWay + 360.0f;
    const bool shortOk = std::fabs(currentCont + shortWay) <= maxTwistDeg;
    const bool longOk = std::fabs(currentCont + longWay) <= maxTwistDeg;
    if (shortOk) {
      delta = shortWay;
    } else if (longOk) {
      delta = longWay;
    } else {
      st.impossible = true;
      return st;
    }
  }

  uint8_t slow = p.slowPct > 100 ? 100 : p.slowPct;
  uint8_t cruise = cruisePct > 100 ? 100 : cruisePct;
  st.dir = delta > 0.0f ? 1 : -1;
  st.speedPct = std::fabs(delta) <= p.slowZoneDeg ? slow : (cruise > slow ? cruise : slow);
  return st;
}
