// Yaw logic without hardware: sensorless twist accounting, endstop
// debouncing, AS5600 angle unwrapping and the goto planner.
#pragma once

#include <stdint.h>

// Speed-weighted travel balance for sensorless twist protection.
// Unit: "ms at 100 %": moving 1000 ms at 50 % adds 500. Calibrating
// ms-per-360 in the same unit makes the estimate hold at any yaw speed.
// Sign: + = right.
class YawBalance {
 public:
  void reset();                   // balance 0, endstop references forgotten
  void restore(int32_t balance);  // value from NVS, references forgotten
  int32_t balance() const;

  // Account dtMs of motion in direction dirSign (+1 right, -1 left, 0 none)
  // at speedPct (0..100) actual output. Sub-ms remainders are kept.
  void integrate(int8_t dirSign, uint8_t speedPct, uint32_t dtMs);

  // True if moving in dirSign is not allowed: balance >= limit (right) or
  // balance <= -limit (left). limit 0 = disabled.
  bool blocked(int8_t dirSign, uint32_t limit) const;

  // Endstop hit on side (-1 left, +1 right). The first hit after reset() /
  // restore() stores the current balance as that side's reference; later
  // hits snap the balance back to it, removing accumulated drift.
  void onEndstop(int8_t side);
  bool hasReference(int8_t side) const;

  // Angle estimate in degrees (continuous) if msPer360 > 0.
  bool estimateDeg(uint32_t msPer360, float& outDeg) const;

 private:
  int64_t accMicro_ = 0;  // balance * 1000
  bool refL_ = false, refR_ = false;
  int64_t refLMicro_ = 0, refRMicro_ = 0;
};

// Debounced active-low switch. update() returns active().
class EndstopDebouncer {
 public:
  void reset(bool rawActive, uint32_t nowMs);
  bool update(bool rawActive, uint32_t nowMs, uint32_t debounceMs);
  bool active() const { return stable_; }
  bool rose() const { return rose_; }  // became active in the last update()

 private:
  bool stable_ = false;
  bool last_ = false;
  bool rose_ = false;
  uint32_t changedAt_ = 0;
};

// Endstop on the wrong side (endstop mode). Switch L must stop logical left
// moves and R logical right ones. If the motor turns the other way (wiring,
// or yawInvert toggled without swapping the switches), the yaw drives into a
// switch that only guards the other direction and nothing stops it. Two
// signs give that away:
//  - a switch closes while the output has been driving away from it for at
//    least kSettleMs (and was not driving toward it just before);
//  - a switch that was closed when a move away from it started is still
//    closed after releaseTravel of driving (balance units, ms at 100 %).
// Then the direction that drove into the switch is refused until reset()
// (yawInvert changed) or until both switches are open again. The other
// direction stays blocked by the closed switch itself, so the user has to
// fix the cause instead of driving on.
struct EndstopSideInputs {
  uint32_t nowMs = 0;
  int8_t drive = 0;      // sign of the actual motor output: -1 left, +1 right, 0 none
  int8_t commanded = 0;  // direction of the running move, 0 = none
  bool lActive = false;  // debounced switch states
  bool rActive = false;
  bool lRose = false;    // became active in this update
  bool rRose = false;
  int32_t balance = 0;          // YawBalance::balance()
  uint32_t releaseTravel = 0;   // travel after which a switch left behind must be open
};

struct EndstopSideResult {
  int8_t wrong = 0;   // direction refused from now on (found in this update), 0 = none
  bool refL = false;  // the edge on that side is a plausible stop: usable as a
  bool refR = false;  // balance reference (YawBalance::onEndstop)
};

class EndstopSideCheck {
 public:
  static constexpr uint32_t kSettleMs = 150;  // debounce + run-down after a reversal

  void reset();
  // A move in dir starts (not called for a repeat of the running one).
  void moveStarted(int8_t dir, bool lActive, bool rActive, int32_t balance);
  // Every tick, after the debouncers.
  EndstopSideResult update(const EndstopSideInputs& in);
  bool blocked(int8_t dir) const { return dir != 0 && dir == refused_; }
  int8_t refused() const { return refused_; }

 private:
  bool recently(int8_t side, uint32_t nowMs) const;

  int8_t refused_ = 0;
  int8_t held_ = 0;  // switch that has to open while the yaw drives away from it
  int32_t heldBal_ = 0;
  bool droveL_ = false, droveR_ = false;  // output ever drove that way
  uint32_t lastL_ = 0, lastR_ = 0;        // last update with the output driving that way
};

float median3(float a, float b, float c);
float wrap360(float deg);                      // [0, 360)
float shortestDelta(float fromAbs, float toAbs);  // (-180, 180]

// Turns absolute 0..360 readings into a continuous angle.
class AngleUnwrapper {
 public:
  // Start from a known continuous angle (e.g. restored from NVS). The first
  // update() picks the continuous value nearest to hintCont whose wrap360
  // equals the reading.
  void reset(float hintCont);
  float update(float absDeg);
  float continuous() const { return cont_; }
  bool initialized() const { return init_; }

 private:
  bool init_ = false;
  float hint_ = 0.0f;
  float lastAbs_ = 0.0f;
  float cont_ = 0.0f;
};

struct GotoParams {
  float windowDeg = 2.0f;    // done when |error| <= window
  float slowZoneDeg = 10.0f; // slow speed when |error| <= slowZone
  uint8_t slowPct = 30;
};

struct GotoStep {
  int8_t dir = 0;         // -1 left, +1 right, 0 stop
  uint8_t speedPct = 0;
  bool done = false;      // inside the window
  bool impossible = false;// both ways would break the twist limit
};

// One planning step. currentCont is the continuous angle, targetAbs in
// [0, 360). Takes the shorter way unless it would end beyond maxTwistDeg
// (|cont| limit, 0 = no limit), then the other way. Speed: cruisePct, or
// slowPct inside the slow zone (never below slowPct).
GotoStep planGoto(float currentCont, float targetAbs, uint8_t cruisePct, float maxTwistDeg,
                  const GotoParams& p);
