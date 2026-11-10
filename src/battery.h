// Battery voltage on ADC1 through the 100k/33k divider. Loop task only.
// Every BAT_SAMPLE_MS: BAT_OVERSAMPLE analogReadMilliVolts() reads averaged
// into one sample, then a BAT_AVG_SAMPLES moving average, then the
// BatteryMonitor thresholds (crit latch with +0.3 V hysteresis).
#pragma once

#include <stdint.h>

#include "settings_model.h"

namespace Battery {
void begin();                           // ADC attenuation 11 dB, first sample at once
void configure(const Settings& s);      // batCal, thresholds
void tick(uint32_t nowMs);
uint32_t mv();       // filtered battery voltage
uint32_t adcMv();    // last raw pin voltage (diagnostics / calibration)
uint8_t pct();
bool low();
bool crit();         // latched
bool present();
}  // namespace Battery
