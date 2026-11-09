#include "battery_math.h"

#include <cmath>

uint32_t batteryMvFromAdc(uint32_t adcMv, float rTop, float rBottom, float cal) {
  if (!(rBottom > 0.0f) || !(rTop >= 0.0f) || !(cal > 0.0f)) return 0;
  double mv = (double)adcMv * ((double)rTop + (double)rBottom) / (double)rBottom * (double)cal;
  if (!std::isfinite(mv) || mv <= 0.0) return 0;
  if (mv >= 4294967295.0) return 0xFFFFFFFFu;
  return (uint32_t)(mv + 0.5);
}

uint8_t batteryPercent(uint32_t mv, uint16_t emptyMv, uint16_t fullMv) {
  if (fullMv <= emptyMv) return mv >= fullMv ? 100 : 0;  // misconfigured: no gradient
  if (mv <= emptyMv) return 0;
  if (mv >= fullMv) return 100;
  uint32_t range = (uint32_t)fullMv - emptyMv;
  return (uint8_t)(((mv - emptyMv) * 100u + range / 2) / range);
}

void BatteryMonitor::reset() {
  low_ = false;
  crit_ = false;
  present_ = true;
  belowCount_ = 0;
}

bool BatteryMonitor::update(uint32_t mv) {
  const bool wasCrit = crit_;
  present_ = mv >= cfg_.presentMinMv;

  if (!present_) {
    // USB power or a fallen-off divider: no new latch, but an existing one
    // stays (a broken sense wire must never unlock the motors).
    belowCount_ = 0;
    low_ = crit_;
    return false;
  }

  if (crit_) {
    if (mv >= (uint32_t)cfg_.critMv + cfg_.critHystMv) {
      crit_ = false;
      belowCount_ = 0;
    }
  } else if (mv < cfg_.critMv) {
    const uint8_t confirm = cfg_.critConfirm ? cfg_.critConfirm : 1;
    if (belowCount_ < 255) belowCount_++;
    if (belowCount_ >= confirm) crit_ = true;
  } else {
    belowCount_ = 0;
  }

  if (crit_ || mv < cfg_.lowMv) {
    low_ = true;
  } else if (mv >= (uint32_t)cfg_.lowMv + cfg_.lowHystMv) {
    low_ = false;
  }
  return crit_ != wasCrit;
}
