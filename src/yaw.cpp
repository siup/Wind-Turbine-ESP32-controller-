// Nacelle yaw: hold-to-run moves with a deadman, twist protection, optional
// endstops or AS5600 absolute encoder, goto, ms-per-360 calibration.
//
// All state is file-local (there is exactly one YawController, `yaw`) and is
// only touched from the loop task.
//
// Direction convention: +1 = right = increasing angle. yawInvert is applied
// by yawMotor at its output stage, so everything here is logical direction.

#include "yaw.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "config.h"
#include "event_log.h"
#include "log.h"
#include "motors.h"
#include "yaw_logic.h"

#if YAW_SENSOR_MODE == YAW_SENSOR_AS5600
#include <Wire.h>
#endif

YawController yaw;

namespace {

constexpr bool kEndstops = YAW_SENSOR_MODE == YAW_SENSOR_ENDSTOPS;
constexpr bool kAs5600 = YAW_SENSOR_MODE == YAW_SENSOR_AS5600;

// AS5600 registers and status bits
constexpr uint8_t kRegStatus = 0x0B;
constexpr uint8_t kRegRawAngle = 0x0C;  // 0x0C high nibble, 0x0D low byte
constexpr uint8_t kStatusMD = 0x20;     // magnet detected
constexpr uint8_t kStatusML = 0x10;     // magnet too weak
constexpr uint8_t kStatusMH = 0x08;     // magnet too strong
constexpr int32_t kCounts = 4096;       // 12-bit angle
constexpr uint32_t kI2cHz = 400000;
// After an I2C error retry slower: a stuck bus costs up to the I2C timeout
// per transaction and must not eat the loop.
constexpr uint32_t kAsRetryMs = 500;
// The stored continuous angle only has to be good to +-180 deg (the
// unwrapper picks the turn nearest to it), so it is saved coarsely.
constexpr float kContSaveDeg = 5.0f;
// Angle moving against the driven direction by this much = sensor reversed.
constexpr float kWrongWayDeg = 15.0f;

struct State {
  // Settings
  uint8_t speedDefault = YAW_SPEED_PCT;
  uint32_t maxBal = YAW_MAX_BALANCE_MS;
  uint32_t ms360 = 0;
  uint16_t maxDeg = 0;
  bool sensRev = false;
  uint32_t holdTimeoutMs = YAW_HOLD_TIMEOUT_MS;

  // Motion
  int8_t dir = 0;          // commanded direction (hold-to-run or goto), 0 = none
  uint8_t speed = 0;       // % of the current / last move
  bool holdArmed = false;  // hold-to-run move running, deadman active
  uint32_t lastHoldMs = 0;

  // Goto
  bool gotoActive = false;
  float gotoTarget = 0.0f;  // [0, 360)
  uint32_t gotoStartMs = 0;

  // ms-per-360 calibration
  bool calActive = false;
  int32_t calStartBal = 0;

  // Sensorless balance (runs in every mode)
  YawBalance bal;
  uint32_t lastIntegrateMs = 0;

  // Endstops
  EndstopDebouncer endL, endR;
  EndstopSideCheck side;  // switch on the wrong side (motor turns the other way)

  // AS5600
  bool asOk = false;        // last read fine and magnet detected
  bool asReported = false;  // first result already logged
  uint8_t asMagnet = 0;     // ML / MH bits last reported
  uint16_t asOff = 0;       // counts (after sensRev) that read as 0 deg
  uint16_t asRaw = 0;       // last filtered raw reading (before sensRev)
  float asAbs = 0.0f;       // [0, 360) after sensRev and offset
  float cont = 0.0f;        // continuous angle, deg
  bool contValid = false;
  bool zeroPending = false; // zero requested while the sensor did not read
  AngleUnwrapper unwrap;
  uint32_t asNextMs = 0;
  float wrongWay = 0.0f;
  float lastCont = 0.0f;
  float contAtLoss = 0.0f;  // re-seed for the unwrapper after an outage
  int32_t balAtLoss = 0;

  // Persistence (NVS namespace "yaw")
  Preferences prefs;
  bool prefsOpen = false;
  bool prefsWarned = false;
  int32_t savedBal = 0;
  uint16_t savedOff = 0;
  float savedCont = 0.0f;
  uint32_t lastSaveMs = 0;
};

State s;

int8_t signOf(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

const char* sideName(int8_t d) { return d < 0 ? "left" : "right"; }

GotoParams gotoParams() {
  GotoParams p;
  p.windowDeg = YAW_GOTO_WINDOW_DEG;
  p.slowZoneDeg = YAW_GOTO_SLOW_DEG;
  p.slowPct = YAW_GOTO_SLOW_PCT;
  return p;
}

uint8_t clampSpeed(uint32_t pct) {
  if (pct < YAW_SPEED_MIN_PCT) return YAW_SPEED_MIN_PCT;
  if (pct > YAW_SPEED_MAX_PCT) return YAW_SPEED_MAX_PCT;
  return (uint8_t)pct;
}

// Direction the motor is being driven in right now (target first, then the
// ramp output while it settles).
int8_t motionDir() {
  int8_t d = signOf(yawMotor.target());
  if (d == 0) d = signOf(yawMotor.actualMilli());
  return d;
}

bool angleValid() { return kAs5600 && s.asOk && s.contValid; }

// L guards logical left, R logical right. A direction that drove into the
// switch on the wrong side stays refused as well (EndstopSideCheck).
bool endstopBlocked(int8_t d) {
  if (!kEndstops || d == 0) return false;
  if (s.side.blocked(d)) return true;
  return d < 0 ? s.endL.active() : s.endR.active();
}

// Twist limit: the AS5600 angle against yawMaxDeg when the angle is valid
// and that limit is set; otherwise the time balance against yawMaxBal (no
// sensor, AS5600 failed, or yawMaxDeg 0 with yawMaxBal set).
bool twistBlocked(int8_t d) {
  if (d == 0) return false;
  if (angleValid() && s.maxDeg > 0) {
    const float lim = (float)s.maxDeg;
    return d > 0 ? s.cont >= lim : s.cont <= -lim;
  }
  return s.bal.blocked(d, s.maxBal);
}

// Travel (balance units) in which a switch left behind has to open.
uint32_t endReleaseTravel() {
  if (s.ms360 == 0) return ENDSTOP_RELEASE_MS;
  const uint32_t t = (uint32_t)(((uint64_t)s.ms360 * ENDSTOP_RELEASE_DEG + 180) / 360);
  return t > ENDSTOP_RELEASE_MIN_MS ? t : ENDSTOP_RELEASE_MIN_MS;
}

bool openPrefs() {
  if (!s.prefsOpen) {
    s.prefsOpen = s.prefs.begin("yaw", false);
    if (!s.prefsOpen && !s.prefsWarned) {
      s.prefsWarned = true;
      EventLog::add('e', "nvs_fail", "yaw namespace");
    }
  }
  return s.prefsOpen;
}

// Persist balance / AS5600 offset / continuous angle if they changed.
// Rate-limited to YAW_BAL_SAVE_MS unless forced.
void persist(uint32_t nowMs, bool force) {
  const int32_t b = s.bal.balance();
  const bool balDirty = b != s.savedBal;
  bool offDirty = false;
  bool contDirty = false;
  bool urgent = false;
  if (kAs5600) {
    offDirty = s.asOff != s.savedOff;
    const float drift = s.contValid ? fabsf(s.cont - s.savedCont) : 0.0f;
    contDirty = s.contValid && drift >= (force ? 0.5f : kContSaveDeg);
    // After a reset the unwrapper picks the turn nearest to the saved
    // value, so it must never lag the truth by 180 deg: a fast yaw skips
    // the rate limit before that.
    urgent = drift >= YAW_CONT_FORCE_SAVE_DEG;
  }
  if (!balDirty && !offDirty && !contDirty) return;
  if (!force && !urgent && (int32_t)(nowMs - s.lastSaveMs) < (int32_t)YAW_BAL_SAVE_MS) return;
  s.lastSaveMs = nowMs;
  if (!openPrefs()) return;
  bool ok = true;
  if (balDirty) {
    ok = s.prefs.putInt("bal", b) > 0 && ok;
    s.savedBal = b;
  }
  if (offDirty) {
    ok = s.prefs.putUShort("asOff", s.asOff) > 0 && ok;
    s.savedOff = s.asOff;
  }
  if (contDirty) {
    ok = s.prefs.putFloat("asCont", s.cont) > 0 && ok;
    s.savedCont = s.cont;
  }
  if (!ok && !s.prefsWarned) {
    s.prefsWarned = true;
    EventLog::add('e', "nvs_fail", "yaw position not saved");
  }
}

// Account the actual motor output since the last call into the balance.
void integrate(uint32_t nowMs) {
  const uint32_t dt = nowMs - s.lastIntegrateMs;
  if ((int32_t)dt <= 0) return;
  s.lastIntegrateMs = nowMs;
  const int32_t m = yawMotor.actualMilli();
  if (m == 0) return;
  uint32_t pct = ((uint32_t)(m < 0 ? -m : m) + 500u) / 1000u;
  if (pct > 100) pct = 100;
  s.bal.integrate(signOf(m), (uint8_t)pct, dt);
}

// Motor stop and motion state reset, no persistence.
void halt(bool brake) {
  if (brake) {
    yawMotor.brake();
  } else {
    yawMotor.stop();
  }
  s.dir = 0;
  s.holdArmed = false;
  s.gotoActive = false;
  s.wrongWay = 0.0f;
}

void stopNow(bool brake) {
  const uint32_t now = millis();
  integrate(now);
  halt(brake);
  persist(now, true);
}

void setDrive(int8_t d, uint8_t pct) {
  if (d != s.dir) {
    s.wrongWay = 0.0f;
    s.lastCont = s.cont;
  }
  s.dir = d;
  s.speed = pct;
  yawMotor.setSpeed((int8_t)(d * (int8_t)pct));
}

uint16_t applyRev(uint16_t raw) {
  return s.sensRev ? (uint16_t)((kCounts - raw) & (kCounts - 1)) : raw;
}

// Position state to zero: balance always, AS5600 offset + continuous angle
// now if the sensor reads, else at its next good reading.
void zeroPosition() {
  s.bal.reset();
  s.calActive = false;
  if (!kAs5600) return;
  if (s.asOk) {
    s.asOff = applyRev(s.asRaw);
    s.asAbs = 0.0f;
    s.cont = 0.0f;
    s.lastCont = 0.0f;
    s.contValid = true;
    s.unwrap.reset(0.0f);
    s.zeroPending = false;
  } else {
    s.zeroPending = true;
  }
}

#if YAW_SENSOR_MODE == YAW_SENSOR_AS5600
bool i2cRead(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // repeated start, executed by requestFrom
  if (Wire.requestFrom((uint8_t)AS5600_I2C_ADDR, n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool readRaw(uint16_t& out) {
  uint8_t b[2];
  if (!i2cRead(kRegRawAngle, b, 2)) return false;
  out = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
  return true;
}

void setSensorState(bool ok, const char* why) {
  if (s.asReported && ok == s.asOk) return;
  const bool first = !s.asReported;
  s.asReported = true;
  if (!ok) {
    if (s.asOk || first) {
      s.contAtLoss = s.cont;
      s.balAtLoss = s.bal.balance();
    }
    s.asOk = false;
    EventLog::add('e', "as5600_err", "%s", why ? why : "?");
    if (s.gotoActive) {
      stopNow(true);
      EventLog::add('w', "goto_abort", "sensor lost");
    }
  } else {
    s.asOk = true;
    EventLog::add('i', "as5600_ok", "angle %d deg", (int)lroundf(s.asAbs));
  }
}
#endif

// One AS5600 cycle: status, 3 raw readings, median, offset, unwrap.
void sampleSensor(uint32_t nowMs) {
#if YAW_SENSOR_MODE == YAW_SENSOR_AS5600
  uint8_t status = 0;
  uint16_t raw[3] = {0, 0, 0};
  const char* why = nullptr;
  if (!i2cRead(kRegStatus, &status, 1)) {
    why = "i2c error";
  } else if (!(status & kStatusMD)) {
    why = "magnet missing";
  } else {
    for (uint16_t& r : raw) {
      if (!readRaw(r)) {
        why = "i2c error";
        break;
      }
    }
  }
  if (why) {
    setSensorState(false, why);
    s.asNextMs = nowMs + kAsRetryMs;
    return;
  }

  const uint8_t magnet = status & (kStatusML | kStatusMH);
  if (magnet != s.asMagnet) {
    s.asMagnet = magnet;
    if (magnet) {
      EventLog::add('w', "as5600_magnet", "%s", (magnet & kStatusML) ? "too weak" : "too strong");
    }
  }

  // Median of 3 across the 0/4095 wrap: deltas relative to the first sample.
  auto rel = [](int32_t d) -> float {
    d = ((d % kCounts) + kCounts) % kCounts;
    if (d >= kCounts / 2) d -= kCounts;
    return (float)d;
  };
  const float med = (float)raw[0] + median3(0.0f, rel((int32_t)raw[1] - raw[0]),
                                            rel((int32_t)raw[2] - raw[0]));
  int32_t r = (int32_t)lroundf(med);
  r = ((r % kCounts) + kCounts) % kCounts;
  s.asRaw = (uint16_t)r;

  bool seeded = false;
  if (s.zeroPending) {
    s.zeroPending = false;
    s.asOff = applyRev(s.asRaw);
    s.unwrap.reset(0.0f);
    s.lastCont = 0.0f;
    seeded = true;
  }

  const uint16_t counts = (uint16_t)((applyRev(s.asRaw) - s.asOff) & (kCounts - 1));
  s.asAbs = wrap360((float)counts * (360.0f / (float)kCounts));

  if (!seeded && !s.asOk && s.asReported) {
    // Back after an outage: the nacelle may have turned meanwhile (driven
    // without the sensor). Seed the turn count from the time balance.
    float hint = s.contAtLoss;
    if (s.ms360 > 0) {
      hint += (float)(s.bal.balance() - s.balAtLoss) * 360.0f / (float)s.ms360;
    }
    s.unwrap.reset(hint);
  }
  s.cont = s.unwrap.update(s.asAbs);
  if (!s.contValid) {
    s.contValid = true;
    s.lastCont = s.cont;
  }
  setSensorState(true, nullptr);
#else
  (void)nowMs;
#endif
}

// Reversed-sensor guard: driving right while the angle falls (or left while
// it rises) would defeat the twist limit and make goto spin forever.
void checkDirection() {
  const int8_t d = motionDir();
  const float dc = s.cont - s.lastCont;
  s.lastCont = s.cont;
  if (d == 0 || s.dir == 0 || d != s.dir) {
    s.wrongWay = 0.0f;
    return;
  }
  s.wrongWay -= dc * (float)d;  // grows when moving against the drive
  if (s.wrongWay < 0.0f) s.wrongWay = 0.0f;
  if (s.wrongWay >= kWrongWayDeg) {
    stopNow(true);
    EventLog::add('e', "yaw_sens_dir", "angle runs backwards, check yawSensRev");
  }
}

void gotoStep() {
  const GotoStep st = planGoto(s.cont, s.gotoTarget, s.speedDefault, (float)s.maxDeg, gotoParams());
  if (st.impossible) {
    stopNow(true);
    EventLog::add('w', "goto_fail", "twist limit both ways");
  } else if (st.done || st.dir == 0) {
    stopNow(true);
    EventLog::add('i', "goto_done", "at %d deg", (int)lroundf(s.asAbs));
  } else {
    setDrive(st.dir, st.speedPct);
  }
}

}  // namespace

// ---------------------------------------------------------------------------

void YawController::begin() {
  // Idempotent: main.cpp may have done this already.
  yawMotor.begin(PIN_YAW_IN1, PIN_YAW_IN2, "yaw");

  const uint32_t now = millis();
  s.lastIntegrateMs = now;
  s.lastSaveMs = now;
  s.lastHoldMs = now;

  if (openPrefs()) {
    const int32_t b = s.prefs.getInt("bal", 0);
    s.bal.restore(b);
    s.savedBal = b;
  }

  if (kEndstops) {
    pinMode(PIN_ENDSTOP_L, INPUT_PULLUP);
    pinMode(PIN_ENDSTOP_R, INPUT_PULLUP);
    s.endL.reset(digitalRead(PIN_ENDSTOP_L) == LOW, now);
    s.endR.reset(digitalRead(PIN_ENDSTOP_R) == LOW, now);
  }

#if YAW_SENSOR_MODE == YAW_SENSOR_AS5600
  if (s.prefsOpen) {
    s.asOff = (uint16_t)(s.prefs.getUShort("asOff", 0) & (kCounts - 1));
    float c = s.prefs.isKey("asCont") ? s.prefs.getFloat("asCont", 0.0f) : 0.0f;
    if (!isfinite(c)) c = 0.0f;
    s.cont = c;
    s.lastCont = c;
    s.savedOff = s.asOff;
    s.savedCont = c;
  }
  s.unwrap.reset(s.cont);
  if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, kI2cHz)) {
    LOGE("yaw: Wire.begin failed");
  }
  Wire.setTimeOut(AS5600_I2C_TIMEOUT_MS);
  // First reading in the first tick(), after configure() has set yawSensRev:
  // a sample taken with the wrong direction would corrupt the turn count.
  s.asNextMs = now;
#endif

  LOGI("yaw: sensor %s, balance %ld", sensorName(), (long)s.bal.balance());
}

void YawController::configure(const Settings& prev, const Settings& next) {
  s.speedDefault = clampSpeed(next.yawSpeed);
  s.maxBal = next.yawMaxBal;
  s.ms360 = next.yawMs360;
  s.maxDeg = next.yawMaxDeg;
  s.holdTimeoutMs = next.fsTimeout > YAW_HOLD_TIMEOUT_MS ? next.fsTimeout : YAW_HOLD_TIMEOUT_MS;

  MotorDriver::Config mc;
  mc.maxDuty = next.yawDuty;
  mc.minPct = 0;
  mc.rampMs = YAW_RAMP_MS;
  mc.invert = next.yawInvert;
  mc.slowDecay = MOTOR_SLOW_DECAY != 0;
  yawMotor.configure(mc);

  const bool invertChanged = prev.yawInvert != next.yawInvert;
  const bool revChanged = kAs5600 && prev.yawSensRev != next.yawSensRev;
  // sensRev must be in place before zeroing: the new offset is taken in the
  // new counting direction.
  s.sensRev = next.yawSensRev;
  if (invertChanged || revChanged) {
    if (!idle()) stopNow(true);
    if (invertChanged) s.side.reset();  // the motor turns the other way now
    zeroPosition();
    persist(millis(), true);
    EventLog::add('w', "yaw_zeroed", "%s changed", invertChanged ? "yawInvert" : "yawSensRev");
  }
}

proto::Err YawController::move(proto::YawDir dir, uint8_t speedPct, uint32_t nowMs) {
  const int8_t d = (int8_t)dir;
  if (d == 0) {
    stop(true);
    return proto::Err::None;
  }
  integrate(nowMs);
  proto::Err e = proto::Err::None;
  if (endstopBlocked(d)) {
    e = proto::Err::Endstop;
  } else if (twistBlocked(d)) {
    e = proto::Err::TwistLimit;
  }
  if (e != proto::Err::None) {
    // The latest intent was "not the old motion": never keep it running.
    if (!idle()) stopNow(true);
    return e;
  }
  s.gotoActive = false;
  if (kEndstops && !(s.holdArmed && s.dir == d)) {
    // A new move (not a repeat of the running one): a switch closed now has
    // to open as the yaw drives away from it.
    s.side.moveStarted(d, s.endL.active(), s.endR.active(), s.bal.balance());
  }
  setDrive(d, clampSpeed(speedPct ? speedPct : s.speedDefault));
  s.holdArmed = true;
  s.lastHoldMs = nowMs;
  return proto::Err::None;
}

void YawController::hold(proto::YawDir dir, uint32_t nowMs) {
  const int8_t d = (int8_t)dir;
  if (d != 0 && s.holdArmed && s.dir == d) s.lastHoldMs = nowMs;
}

proto::Err YawController::gotoAngle(float deg, uint32_t nowMs) {
  if (!hasAngleSensor()) return proto::Err::NoSensor;
  if (!isfinite(deg)) return proto::Err::OutOfRange;
  const float target = wrap360(deg);
  const GotoStep st = planGoto(s.cont, target, s.speedDefault, (float)s.maxDeg, gotoParams());
  if (st.impossible) return proto::Err::TwistLimit;
  integrate(nowMs);
  if (st.done || st.dir == 0) {
    // Already there: just make sure nothing keeps moving.
    if (!idle()) stopNow(true);
    return proto::Err::None;
  }
  s.holdArmed = false;
  s.gotoActive = true;
  s.gotoTarget = target;
  s.gotoStartMs = nowMs;
  setDrive(st.dir, st.speedPct);
  return proto::Err::None;
}

void YawController::stop(bool brake) { stopNow(brake); }

proto::Err YawController::zero() {
  if (!idle()) return proto::Err::Busy;
  zeroPosition();
  persist(millis(), true);
  EventLog::add('i', "yaw_zero", "position set to 0");
  return proto::Err::None;
}

proto::Err YawController::calibrate(proto::YawAction action, uint32_t* resultMs) {
  if (kAs5600) return proto::Err::NoSensor;
  switch (action) {
    case proto::YawAction::CalStart:
      s.calActive = true;
      s.calStartBal = s.bal.balance();
      return proto::Err::None;
    case proto::YawAction::CalCancel:
      s.calActive = false;
      return proto::Err::None;
    case proto::YawAction::CalStop: {
      if (!s.calActive) return proto::Err::OutOfRange;
      s.calActive = false;
      integrate(millis());
      const int64_t diff = (int64_t)s.bal.balance() - (int64_t)s.calStartBal;
      const uint64_t d = (uint64_t)(diff < 0 ? -diff : diff);
      double maxMs = 600000.0;
      const FieldDesc* f = settingsFindField("yawMs360");
      if (f) maxMs = f->max;
      if (d < YAW_CAL_MIN_MS || (double)d > maxMs) return proto::Err::OutOfRange;
      if (resultMs) *resultMs = (uint32_t)d;
      EventLog::add('i', "yaw_cal", "%lu ms per turn", (unsigned long)d);
      return proto::Err::None;
    }
    default:
      return proto::Err::BadType;
  }
}

void YawController::tick(uint32_t nowMs) {
  // Ramp + PWM of the yaw motor. Runs at most once per MOTOR_TICK_MS, so an
  // extra yawMotor.tick() from main.cpp in the same pass is a no-op.
  yawMotor.tick(nowMs);
  integrate(nowMs);

  // Somebody stopped the motor under us (should not happen, but a stale
  // "moving" state would let a late hold restart nothing; keep it honest).
  if (s.dir != 0 && yawMotor.target() == 0) {
    s.dir = 0;
    s.holdArmed = false;
    s.gotoActive = false;
  }

  if (kEndstops) {
    s.endL.update(digitalRead(PIN_ENDSTOP_L) == LOW, nowMs, ENDSTOP_DEBOUNCE_MS);
    s.endR.update(digitalRead(PIN_ENDSTOP_R) == LOW, nowMs, ENDSTOP_DEBOUNCE_MS);
    EndstopSideInputs in;
    in.nowMs = nowMs;
    in.drive = signOf(yawMotor.actualMilli());
    in.commanded = s.dir;
    in.lActive = s.endL.active();
    in.rActive = s.endR.active();
    in.lRose = s.endL.rose();
    in.rRose = s.endR.rose();
    in.balance = s.bal.balance();
    in.releaseTravel = endReleaseTravel();
    const EndstopSideResult r = s.side.update(in);
    // Only a plausible hit corrects the balance (no bogus reference).
    if (r.refL) s.bal.onEndstop(-1);
    if (r.refR) s.bal.onEndstop(+1);
    if (r.wrong != 0) {
      stopNow(true);
      EventLog::add('e', "yaw_end_dir", "switch %c while going %s, both ways blocked",
                    r.wrong < 0 ? 'R' : 'L', sideName(r.wrong));
    }
  }

  if (kAs5600 && (int32_t)(nowMs - s.asNextMs) >= 0) {
    s.asNextMs = nowMs + AS5600_PERIOD_MS;
    sampleSensor(nowMs);  // may push asNextMs further on error
    if (angleValid()) {
      checkDirection();
      if (s.gotoActive) gotoStep();
    }
  }

  if (s.gotoActive && (int32_t)(nowMs - s.gotoStartMs) >= (int32_t)YAW_GOTO_TIMEOUT_MS) {
    stopNow(true);
    EventLog::add('w', "goto_timeout", "%d deg short of %d",
                  (int)lroundf(fabsf(shortestDelta(s.asAbs, s.gotoTarget))),
                  (int)lroundf(s.gotoTarget));
  }

  if (s.holdArmed && (int32_t)(nowMs - s.lastHoldMs) > (int32_t)s.holdTimeoutMs) {
    stopNow(true);
    EventLog::add('w', "yaw_deadman", "no hold for %lu ms", (unsigned long)s.holdTimeoutMs);
  }

  const int8_t md = motionDir();
  if (md != 0) {
    if (endstopBlocked(md)) {
      stopNow(true);
      EventLog::add('i', "endstop", "%s", sideName(md));
    } else if (twistBlocked(md)) {
      stopNow(true);
      if (angleValid() && s.maxDeg > 0) {  // which limit tripped (see twistBlocked)
        EventLog::add('w', "twist_limit", "%s at %ld deg", sideName(md), (long)lroundf(s.cont));
      } else {
        EventLog::add('w', "twist_limit", "%s, balance %ld", sideName(md), (long)s.bal.balance());
      }
    }
  }

  persist(nowMs, false);
}

void YawController::fill(TelemetrySnapshot& t) const {
  t.yawMoving = (proto::YawDir)motionDir();
  t.yawSpeed = s.speed;

  const bool av = angleValid();
  t.hasAngle = av;
  t.angle = av ? (int16_t)(lroundf(s.asAbs) % 360) : 0;

  const bool twL = twistBlocked(-1);
  const bool twR = twistBlocked(+1);
  t.limitL = twL || endstopBlocked(-1);
  t.limitR = twR || endstopBlocked(+1);
  t.twist = twL ? -1 : (twR ? 1 : 0);

  t.bal = s.bal.balance();
  t.balMax = s.maxBal;

  if (av) {
    t.hasPos = true;
    t.pos = (int32_t)lroundf(s.cont);
    t.posEst = false;
    t.hasPosMax = s.maxDeg > 0;
    t.posMax = s.maxDeg;
  } else {
    float est = 0.0f;
    t.hasPos = s.ms360 > 0 && s.bal.estimateDeg(s.ms360, est);
    t.pos = t.hasPos ? (int32_t)lroundf(est) : 0;
    t.posEst = true;
    t.hasPosMax = s.maxBal > 0 && s.ms360 > 0;
    t.posMax = t.hasPosMax ? (int32_t)(((uint64_t)s.maxBal * 360u + s.ms360 / 2) / s.ms360) : 0;
  }

  t.gotoActive = s.gotoActive;
  t.gotoTarget = s.gotoActive ? (int16_t)(lroundf(s.gotoTarget) % 360) : 0;
  t.sensor = sensorName();
  if (kAs5600) {
    t.sensorOk = s.asOk;
  } else if (kEndstops) {
    // both closed = wiring fault; a switch on the wrong side = yawInvert or wiring
    t.sensorOk = !(s.endL.active() && s.endR.active()) && s.side.refused() == 0;
  } else {
    t.sensorOk = true;
  }
  t.calActive = s.calActive;
}

bool YawController::idle() const { return yawMotor.idle() && !s.gotoActive; }

bool YawController::hasAngleSensor() const { return angleValid(); }

const char* YawController::sensorName() const {
  return kAs5600 ? "as5600" : (kEndstops ? "endstops" : "none");
}

void YawController::saveNow() {
  const uint32_t now = millis();
  integrate(now);
  persist(now, true);
}
