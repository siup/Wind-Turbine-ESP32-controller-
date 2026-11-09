// DRV8833 motor outputs: LEDC PWM on IN1/IN2, shared SLEEP pin.
//
// Truth table of one DRV8833 bridge:
//   IN1 IN2
//    0   0   coast (outputs Hi-Z)
//    1   0   forward
//    0   1   reverse
//    1   1   brake (slow decay)
// Fast decay PWM drives one input and holds the other LOW; slow decay holds
// one input HIGH and PWMs the other with the inverted duty. Zero output is
// always coast (both LOW), never brake.

#include "motors.h"

#include <Arduino.h>

#include "config.h"
#include "event_log.h"
#include "log.h"

MotorDriver rotorMotor;
MotorDriver yawMotor;

namespace {

// ledcWrite() turns the all-ones value into a true 100 % (it writes
// max + 1 internally, see esp32-hal-ledc.c), so 255 at 8 bits = always HIGH.
constexpr uint16_t kFullDuty = (1u << MOTOR_PWM_BITS) - 1;

// Longest dt fed to the ramp in one step. A loop stall (flash write, WiFi
// burst) must not turn a soft start into a jump.
constexpr uint32_t kMaxRampDtMs = 10 * MOTOR_TICK_MS;

constexpr int32_t kForceWrite = 0x7fffffff;

}  // namespace

// ---------------------------------------------------------------------------
// Early init
// ---------------------------------------------------------------------------

void motorsEarlyInit() {
  const uint8_t pins[] = {PIN_ROTOR_IN1, PIN_ROTOR_IN2, PIN_YAW_IN1, PIN_YAW_IN2, PIN_DRV_SLEEP};
  for (uint8_t pin : pins) {
    // Output latch LOW before the driver is enabled, so the pin never
    // drives HIGH for even a moment.
    gpio_set_level((gpio_num_t)pin, 0);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

// ---------------------------------------------------------------------------
// DriverPower: DRV8833 nSLEEP
// ---------------------------------------------------------------------------

// Every MotorDriver::tick() reports its own activity here, so the bridge
// sleeps after DRIVER_SLEEP_IDLE_MS without any motor active even if nobody
// calls DriverPower::tick(); calling it as well is harmless.
namespace {
bool g_awake = false;
uint32_t g_lastActiveMs = 0;

void noteActive(uint32_t nowMs) {
  // Never move the timestamp backwards (wake() stores millis(), callers pass
  // a loop timestamp that may be a little older).
  if ((int32_t)(nowMs - g_lastActiveMs) > 0) g_lastActiveMs = nowMs;
}

void sleepIfIdle(uint32_t nowMs) {
  if (g_awake && (int32_t)(nowMs - g_lastActiveMs) >= (int32_t)DRIVER_SLEEP_IDLE_MS) {
    digitalWrite(PIN_DRV_SLEEP, LOW);
    g_awake = false;
  }
}
}  // namespace

namespace DriverPower {

void begin() {
  digitalWrite(PIN_DRV_SLEEP, HIGH);
  g_awake = true;
  g_lastActiveMs = millis();
}

void wake() {
  if (!g_awake) {
    digitalWrite(PIN_DRV_SLEEP, HIGH);
    g_awake = true;
  }
  // Restart the idle timer so a wake from sleep is not undone by the very
  // next tick.
  g_lastActiveMs = millis();
}

void tick(uint32_t nowMs, bool anyActive) {
  if (anyActive) {
    if (!g_awake) wake();
    noteActive(nowMs);
  } else {
    sleepIfIdle(nowMs);
  }
}

bool awake() { return g_awake; }

}  // namespace DriverPower

// ---------------------------------------------------------------------------
// MotorDriver
// ---------------------------------------------------------------------------

void MotorDriver::begin(uint8_t pinIn1, uint8_t pinIn2, const char* tag) {
  if (begun_) return;  // idempotent: a second begin() must not re-attach LEDC
  begun_ = true;
  in1_ = pinIn1;
  in2_ = pinIn2;
  tag_ = tag ? tag : "";
  const uint8_t pins[] = {in1_, in2_};
  for (uint8_t pin : pins) {
    if (!ledcAttach(pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS)) {
      // The pin stays a plain GPIO output LOW from motorsEarlyInit(): the
      // motor cannot run, which is the safe failure.
      LOGE("%s: ledcAttach(%u) failed", tag_, (unsigned)pin);
      EventLog::add('e', "pwm_fail", "%s pin %u", tag_, (unsigned)pin);
    }
  }
  cfg_.slowDecay = cfg_.slowDecay || (MOTOR_SLOW_DECAY != 0);
  ramp_.setFullScaleMs(cfg_.rampMs);
  ramp_.jumpTo(0);
  braking_ = false;
  duty1_ = duty2_ = 0xffff;
  writePins(0, 0);
  lastWritten_ = 0;
  lastTickMs_ = millis();
}

void MotorDriver::configure(const Config& c) {
  const bool invert = cfg_.invert;
  cfg_ = c;
  cfg_.invert = invert;  // handled below
  // MOTOR_SLOW_DECAY is a global compile-time switch.
  cfg_.slowDecay = c.slowDecay || (MOTOR_SLOW_DECAY != 0);
  if (c.invert == cfg_.invert) {
    hasPendingInvert_ = false;
  } else if (idle()) {
    cfg_.invert = c.invert;
    hasPendingInvert_ = false;
  } else {
    // Swapping the direction under a running motor would slam it into
    // reverse. Applied as soon as it is idle.
    pendingInvert_ = c.invert;
    hasPendingInvert_ = true;
  }
  if (overrideRampMs_ == 0) ramp_.setFullScaleMs(cfg_.rampMs);
  lastWritten_ = kForceWrite;  // new duty mapping takes effect on the next tick
}

void MotorDriver::setSpeed(int8_t pct) {
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  const int32_t m = (int32_t)pct * 1000;
  // Keep a running override ramp (failsafe ramp-down) if the target does not
  // change: a "speed 0" right after a failsafe trip must not slow the stop.
  if (overrideRampMs_ != 0 && m != ramp_.targetMilli()) {
    overrideRampMs_ = 0;
    ramp_.setFullScaleMs(cfg_.rampMs);
  }
  if (braking_) {
    braking_ = false;
    lastWritten_ = kForceWrite;
  }
  // Wake the bridge with the command, not with the first PWM step: the
  // DRV8833 needs up to 1 ms after nSLEEP goes HIGH.
  if (m != 0) DriverPower::wake();
  ramp_.setTargetMilli(m);
}

void MotorDriver::setSpeedWithRamp(int8_t pct, uint32_t rampMs) {
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  if (braking_) {
    braking_ = false;
    lastWritten_ = kForceWrite;
  }
  overrideRampMs_ = rampMs ? rampMs : 1;  // 0 would mean "no override"; 1 ms = instant
  ramp_.setFullScaleMs(overrideRampMs_);
  if (pct != 0) DriverPower::wake();
  ramp_.setTargetMilli((int32_t)pct * 1000);
}

void MotorDriver::stop() {
  braking_ = false;
  overrideRampMs_ = 0;
  ramp_.setFullScaleMs(cfg_.rampMs);
  ramp_.jumpTo(0);
  applyPendingInvert();
  writePins(0, 0);
  lastWritten_ = 0;
}

void MotorDriver::brake() {
  overrideRampMs_ = 0;
  ramp_.setFullScaleMs(cfg_.rampMs);
  ramp_.jumpTo(0);
  applyPendingInvert();
  lastWritten_ = 0;
  if (braking_) return;  // already braking, bridge awake: nothing to do
  braking_ = true;
  DriverPower::wake();
  writePins(kFullDuty, kFullDuty);
}

void MotorDriver::tick(uint32_t nowMs) {
  uint32_t dt = nowMs - lastTickMs_;
  if ((int32_t)dt < (int32_t)MOTOR_TICK_MS) return;
  lastTickMs_ = nowMs;
  if (dt > kMaxRampDtMs) dt = kMaxRampDtMs;

  // A braking motor counts as idle for the sleep timer.
  if (idle()) {
    sleepIfIdle(nowMs);
  } else {
    noteActive(nowMs);
  }

  if (braking_) {
    // The brake ends when the bridge goes to sleep (outputs Hi-Z anyway);
    // release the inputs so a later wake does not re-engage it.
    if (!DriverPower::awake()) {
      braking_ = false;
      writePins(0, 0);
      lastWritten_ = 0;
    }
    return;
  }

  ramp_.update(dt);
  if (overrideRampMs_ != 0 && ramp_.settled()) {
    overrideRampMs_ = 0;
    ramp_.setFullScaleMs(cfg_.rampMs);
  }
  if (hasPendingInvert_ && idle()) applyPendingInvert();
  writeOutput(ramp_.valueMilli());
}

int8_t MotorDriver::target() const { return (int8_t)ramp_.target(); }
int8_t MotorDriver::actual() const { return (int8_t)ramp_.value(); }
int32_t MotorDriver::actualMilli() const { return ramp_.valueMilli(); }
bool MotorDriver::idle() const { return ramp_.targetMilli() == 0 && ramp_.valueMilli() == 0; }
bool MotorDriver::braking() const { return braking_; }

void MotorDriver::applyPendingInvert() {
  if (!hasPendingInvert_) return;
  cfg_.invert = pendingInvert_;
  hasPendingInvert_ = false;
  lastWritten_ = kForceWrite;
}

void MotorDriver::writeOutput(int32_t milli) {
  if (braking_ || milli == lastWritten_) return;
  lastWritten_ = milli;

  const int32_t m = cfg_.invert ? -milli : milli;
  const uint32_t mag = (uint32_t)(m < 0 ? -m : m);
  if (mag == 0) {
    writePins(0, 0);  // coast
    return;
  }

  // duty = minDuty + (maxDuty - minDuty) * |p| / 100, minDuty = maxDuty * minPct / 100.
  // Computed in 1/100 duty steps from the milli-percent value.
  const uint32_t max100 = (uint32_t)cfg_.maxDuty * 100u;
  uint32_t min100 = (uint32_t)cfg_.maxDuty * (cfg_.minPct > 100 ? 100u : cfg_.minPct);
  if (min100 > max100) min100 = max100;
  const uint32_t d100 = min100 + (uint32_t)(((uint64_t)(max100 - min100) * mag) / (uint32_t)Ramp::kFull);
  uint32_t duty = (d100 + 50u) / 100u;
  if (duty > kFullDuty) duty = kFullDuty;
  if (duty == 0) {
    writePins(0, 0);  // below one PWM step: coast, never brake
    return;
  }

  DriverPower::wake();
  const uint16_t d = (uint16_t)duty;
  if (!cfg_.slowDecay) {
    if (m > 0) {
      writePins(d, 0);
    } else {
      writePins(0, d);
    }
  } else {
    // Drive while the PWM input is LOW, brake while it is HIGH.
    const uint16_t inv = (uint16_t)(kFullDuty - d);
    if (m > 0) {
      writePins(kFullDuty, inv);
    } else {
      writePins(inv, kFullDuty);
    }
  }
}

void MotorDriver::writePins(uint16_t d1, uint16_t d2) {
  if (!begun_) return;
  // Lower a falling IN1 before raising IN2, so a direction change passes
  // through coast instead of a brief brake.
  if (d1 < duty1_) {
    ledcWrite(in1_, d1);
    duty1_ = d1;
  }
  if (d2 != duty2_) {
    ledcWrite(in2_, d2);
    duty2_ = d2;
  }
  if (d1 != duty1_) {
    ledcWrite(in1_, d1);
    duty1_ = d1;
  }
}
