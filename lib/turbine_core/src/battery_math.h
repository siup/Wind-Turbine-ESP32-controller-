// Battery measurement math and the low / critical threshold state machine.
#pragma once

#include <stdint.h>

template <uint8_t N>
class MovingAverage {
 public:
  void reset() {
    count_ = 0;
    head_ = 0;
    sum_ = 0;
  }
  void push(uint32_t v) {
    if (count_ == N) {
      sum_ -= buf_[head_];
    } else {
      count_++;
    }
    buf_[head_] = v;
    sum_ += v;
    head_ = (uint8_t)((head_ + 1) % N);
  }
  uint32_t mean() const { return count_ ? (uint32_t)((sum_ + count_ / 2) / count_) : 0; }
  uint8_t count() const { return count_; }
  bool full() const { return count_ == N; }

 private:
  uint32_t buf_[N] = {};
  uint64_t sum_ = 0;
  uint8_t count_ = 0;
  uint8_t head_ = 0;
};

// Battery voltage from the ADC pin voltage through a rTop/rBottom divider.
uint32_t batteryMvFromAdc(uint32_t adcMv, float rTop, float rBottom, float cal);

// Linear 0..100 % between emptyMv and fullMv, clamped.
uint8_t batteryPercent(uint32_t mv, uint16_t emptyMv, uint16_t fullMv);

// Threshold state machine fed with filtered readings.
//  present: mv >= presentMinMv. While absent, low is false and a new crit
//           cannot latch, but an existing crit latch is never cleared by
//           absence (a fallen-off divider wire must not unlock the motors).
//  crit:    latches after critConfirm consecutive readings below critMv;
//           releases only when mv >= critMv + critHystMv.
//  low:     set below lowMv (or while crit), cleared at lowMv + lowHystMv.
class BatteryMonitor {
 public:
  struct Config {
    uint16_t lowMv = 7000;
    uint16_t critMv = 6200;
    uint16_t critHystMv = 300;
    uint16_t lowHystMv = 100;
    uint8_t critConfirm = 2;
    uint16_t presentMinMv = 2500;
  };
  void configure(const Config& c) { cfg_ = c; }
  const Config& config() const { return cfg_; }
  void reset();
  // Returns true if the crit latch changed state.
  bool update(uint32_t mv);
  bool low() const { return low_; }
  bool crit() const { return crit_; }
  bool present() const { return present_; }

 private:
  Config cfg_;
  bool low_ = false;
  bool crit_ = false;
  bool present_ = true;
  uint8_t belowCount_ = 0;
};
