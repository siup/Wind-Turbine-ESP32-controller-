// DRV8833 motor outputs. One MotorDriver per H-bridge (IN1/IN2 PWM via LEDC,
// 20 kHz, 8 bit), a shared SLEEP pin handled by DriverPower. Loop task only.
#pragma once

#include <stdint.h>

#include "ramp.h"

// First line of setup(): IN pins and SLEEP as OUTPUT LOW before anything
// else runs, so the motors cannot twitch during boot.
void motorsEarlyInit();

namespace DriverPower {
void begin();   // SLEEP HIGH (awake)
void wake();    // SLEEP HIGH now; called by MotorDriver before any non-zero output
// SLEEP LOW once anyActive has been false for DRIVER_SLEEP_IDLE_MS.
void tick(uint32_t nowMs, bool anyActive);
bool awake();
}  // namespace DriverPower

class MotorDriver {
 public:
  struct Config {
    uint8_t maxDuty = 255;   // PWM ceiling at 100 %
    uint8_t minPct = 0;      // dead-band: 1 % maps to minPct % of maxDuty
    uint32_t rampMs = 500;   // full-scale ramp time
    bool invert = false;     // swap direction (applied when idle, deferred otherwise)
    bool slowDecay = false;  // MOTOR_SLOW_DECAY
  };

  void begin(uint8_t pinIn1, uint8_t pinIn2, const char* tag);
  void configure(const Config& c);

  // Ramped speed, -100..100 (sign = direction). Duty mapping for |p| > 0:
  // minDuty + (maxDuty - minDuty) * |p| / 100 with minDuty = maxDuty * minPct / 100,
  // computed from the milli-percent ramp value for smooth steps.
  void setSpeed(int8_t pct);
  // Same, but this ramp only uses rampMs (used for the failsafe ramp-down).
  void setSpeedWithRamp(int8_t pct, uint32_t rampMs);
  // Coast now: both inputs LOW, target = actual = 0.
  void stop();
  // Brake now: both inputs HIGH (full duty), target = actual = 0. Lasts until
  // the next setSpeed() or until DriverPower puts the bridge to sleep.
  void brake();

  // Advances the ramp every MOTOR_TICK_MS and writes the PWM.
  void tick(uint32_t nowMs);

  int8_t target() const;        // percent
  int8_t actual() const;        // percent, ramp output
  int32_t actualMilli() const;  // milli-percent, ramp output
  bool idle() const;            // target 0 and actual 0
  bool braking() const;

 private:
  void writeOutput(int32_t milli);
  uint8_t in1_ = 0, in2_ = 0;
  const char* tag_ = "";
  Config cfg_;
  bool pendingInvert_ = false;
  bool hasPendingInvert_ = false;
  Ramp ramp_;
  uint32_t overrideRampMs_ = 0;  // 0 = none
  uint32_t lastTickMs_ = 0;
  bool braking_ = false;
  int32_t lastWritten_ = 0x7fffffff;
};

extern MotorDriver rotorMotor;
extern MotorDriver yawMotor;
