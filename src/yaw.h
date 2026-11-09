// Nacelle yaw control over yawMotor. Hold-to-run moves with a deadman,
// optional sensors selected by YAW_SENSOR_MODE in config.h. Loop task only.
//
// Persisted position state lives in NVS namespace "yaw" (keys "bal",
// "asOff", "asCont"), separate from settings so a factory reset keeps it.
#pragma once

#include <stdint.h>

#include "frames.h"
#include "protocol.h"
#include "settings_model.h"

class YawController {
 public:
  // Sensor init (endstop pins INPUT_PULLUP / Wire with AS5600_I2C_TIMEOUT_MS),
  // restore balance and AS5600 continuous angle from NVS.
  void begin();

  // Speed default, duty, invert, twist limits, ms-per-360. A change of
  // yawInvert (or yawSensRev in AS5600 mode) zeroes the position state and
  // logs a warning, because the stored balance would point the wrong way.
  void configure(const Settings& prev, const Settings& next);

  // Start a hold-to-run move (dir Stop = stop with brake). speedPct 0 = the
  // yawSpeed setting. Cancels goto. Checks endstop and twist limit first
  // (Err::Endstop / Err::TwistLimit). Arms the deadman.
  proto::Err move(proto::YawDir dir, uint8_t speedPct, uint32_t nowMs);

  // Deadman refresh: only extends a move already running in `dir`. Ignored
  // after stop(), a failsafe, a limit stop or a direction change, so a late
  // repeat can never restart the motor.
  void hold(proto::YawDir dir, uint32_t nowMs);

  // Absolute target in degrees; Err::NoSensor unless AS5600 is present and
  // the magnet is OK; Err::TwistLimit if both ways break yawMaxDeg.
  proto::Err gotoAngle(float deg, uint32_t nowMs);

  // Stop now (brake or coast), cancel goto, disarm the deadman, persist
  // the balance if it changed.
  void stop(bool brake);

  // Current position becomes 0 (balance = 0 / AS5600 offset). Err::Busy if moving.
  proto::Err zero();

  // ms-per-360 measurement (NONE / ENDSTOPS modes): CalStart remembers the
  // balance, the user drives one full turn, CalStop stores
  // |balance - start| in *resultMs (>= YAW_CAL_MIN_MS, else Err::OutOfRange).
  // In AS5600 mode returns Err::NoSensor (not needed there).
  proto::Err calibrate(proto::YawAction action, uint32_t* resultMs);

  // Deadman (max(YAW_HOLD_TIMEOUT_MS, fsTimeout)), limit checks while moving,
  // balance integration, endstop debounce, AS5600 sampling + goto control,
  // goto timeout, periodic balance save (YAW_BAL_SAVE_MS, only if changed).
  void tick(uint32_t nowMs);

  // Fill the yaw part of a telemetry snapshot.
  void fill(TelemetrySnapshot& t) const;

  bool idle() const;          // motor idle and no goto running
  bool hasAngleSensor() const;// AS5600 mode with magnet OK
  const char* sensorName() const;  // "none" | "endstops" | "as5600"
  void saveNow();             // persist position state immediately
};

extern YawController yaw;
