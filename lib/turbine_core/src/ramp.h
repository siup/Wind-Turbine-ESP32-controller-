// Rate-limited setpoint, kept in milli-percent so that an 8-bit PWM still
// gets a smooth staircase (500 ms full-scale ramp = 2 % per 10 ms tick).
#pragma once

#include <stdint.h>

class Ramp {
 public:
  static constexpr int32_t kFull = 100000;  // 100 % in milli-percent

  // Time to travel the full 0..100 % range. 0 = no ramp (jump).
  void setFullScaleMs(uint32_t ms) { fullScaleMs_ = ms; }
  uint32_t fullScaleMs() const { return fullScaleMs_; }

  // Target in percent, clamped to -100..100.
  void setTarget(int32_t pct) { setTargetMilli(pct * 1000); }
  void setTargetMilli(int32_t m) {
    if (m > kFull) m = kFull;
    if (m < -kFull) m = -kFull;
    if (m != target_) rem_ = 0;
    target_ = m;
  }

  // Set value and target at once, no ramp.
  void jumpTo(int32_t pct) {
    setTarget(pct);
    value_ = target_;
    rem_ = 0;
  }

  // Advance by dtMs. Returns true if the value changed.
  bool update(uint32_t dtMs) {
    if (value_ == target_) {
      rem_ = 0;
      return false;
    }
    if (fullScaleMs_ == 0) {
      value_ = target_;
      rem_ = 0;
      return true;
    }
    uint64_t num = (uint64_t)dtMs * (uint64_t)kFull + rem_;
    uint64_t step64 = num / fullScaleMs_;
    rem_ = (uint32_t)(num % fullScaleMs_);
    if (step64 == 0) return false;
    int64_t diff = (int64_t)target_ - (int64_t)value_;
    int64_t mag = diff < 0 ? -diff : diff;
    if ((uint64_t)mag <= step64) {
      value_ = target_;
      rem_ = 0;
    } else {
      value_ += (int32_t)(diff < 0 ? -(int64_t)step64 : (int64_t)step64);
    }
    return true;
  }

  int32_t valueMilli() const { return value_; }
  int32_t targetMilli() const { return target_; }
  // Percent rounded half away from zero.
  int32_t value() const { return value_ >= 0 ? (value_ + 500) / 1000 : (value_ - 500) / 1000; }
  int32_t target() const { return target_ >= 0 ? (target_ + 500) / 1000 : (target_ - 500) / 1000; }
  bool settled() const { return value_ == target_; }

 private:
  int32_t value_ = 0;
  int32_t target_ = 0;
  uint32_t fullScaleMs_ = 500;
  uint32_t rem_ = 0;  // carry of the integer division, keeps long ramps exact
};
