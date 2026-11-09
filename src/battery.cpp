// Battery voltage on ADC1 through the 100k/33k divider.
//
// The moving average holds pin millivolts, so a new batCal applies to the
// very next reading of mv(). A change of presence (battery plugged in or
// pulled while on USB) restarts the average: mixing "absent" and "present"
// samples would sweep the filtered value through the critical band and
// latch a false crit.

#include "battery.h"

#include <Arduino.h>

#include "battery_math.h"
#include "config.h"
#include "event_log.h"
#include "log.h"

namespace {

MovingAverage<BAT_AVG_SAMPLES> g_avg;
BatteryMonitor g_mon;
float g_cal = BAT_CAL;
uint16_t g_emptyMv = BAT9V_EMPTY_MV;
uint16_t g_fullMv = BAT9V_FULL_MV;
uint32_t g_mv = 0;
uint32_t g_adcMv = 0;
uint8_t g_pct = 0;
uint32_t g_nextMs = 0;
bool g_samplePresent = false;  // presence of the last raw sample
bool g_wasLow = false;
bool g_wasPresent = true;      // BatteryMonitor starts as "present"

void sample() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < BAT_OVERSAMPLE; i++) sum += analogReadMilliVolts(PIN_BAT_ADC);
  g_adcMv = (sum + BAT_OVERSAMPLE / 2) / BAT_OVERSAMPLE;

  const bool present =
      batteryMvFromAdc(g_adcMv, BAT_DIV_R_TOP, BAT_DIV_R_BOTTOM, g_cal) >= BAT_PRESENT_MIN_MV;
  if (present != g_samplePresent) {
    g_avg.reset();
    g_samplePresent = present;
  }
  g_avg.push(g_adcMv);
  g_mv = batteryMvFromAdc(g_avg.mean(), BAT_DIV_R_TOP, BAT_DIV_R_BOTTOM, g_cal);

  const bool critChanged = g_mon.update(g_mv);
  g_pct = g_mon.present() ? batteryPercent(g_mv, g_emptyMv, g_fullMv) : 0;

  if (g_mon.present() != g_wasPresent) {
    g_wasPresent = g_mon.present();
    if (g_wasPresent) {
      EventLog::add('i', "bat_present", "%lu mV", (unsigned long)g_mv);
    } else {
      EventLog::add('w', "bat_absent", "%lu mV, USB power?", (unsigned long)g_mv);
    }
  }
  if (critChanged) {
    if (g_mon.crit()) {
      EventLog::add('e', "bat_crit", "%lu mV, motors locked", (unsigned long)g_mv);
    } else {
      EventLog::add('i', "bat_ok", "%lu mV, crit released", (unsigned long)g_mv);
    }
  }
  // Low is also set while crit; the crit event already covers that case.
  if (g_mon.low() && !g_wasLow && !g_mon.crit()) {
    EventLog::add('w', "bat_low", "%lu mV", (unsigned long)g_mv);
  }
  g_wasLow = g_mon.low();
}

}  // namespace

namespace Battery {

void begin() {
  // The core configures the channel on the first read (default 11 dB);
  // analogSetPinAttenuation() only works on an initialised ADC pin, so read
  // once first, then pin the attenuation explicitly (~0-3.1 V range).
  (void)analogReadMilliVolts(PIN_BAT_ADC);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);

  BatteryMonitor::Config c;
  c.lowMv = (BAT_PRESET == BAT_PRESET_2S) ? BAT2S_LOW_MV : BAT9V_LOW_MV;
  c.critMv = (BAT_PRESET == BAT_PRESET_2S) ? BAT2S_CRIT_MV : BAT9V_CRIT_MV;
  c.critHystMv = BAT_CRIT_HYST_MV;
  c.lowHystMv = BAT_LOW_HYST_MV;
  c.critConfirm = BAT_CRIT_CONFIRM;
  c.presentMinMv = BAT_PRESENT_MIN_MV;
  g_mon.configure(c);
  g_mon.reset();
  g_emptyMv = (BAT_PRESET == BAT_PRESET_2S) ? BAT2S_EMPTY_MV : BAT9V_EMPTY_MV;
  g_fullMv = (BAT_PRESET == BAT_PRESET_2S) ? BAT2S_FULL_MV : BAT9V_FULL_MV;
  g_avg.reset();
  g_wasLow = false;
  g_wasPresent = g_mon.present();
  g_samplePresent = false;

  sample();
  g_nextMs = millis() + BAT_SAMPLE_MS;
  LOGI("battery: %lu mV (pin %lu mV)%s", (unsigned long)g_mv, (unsigned long)g_adcMv,
       g_mon.present() ? "" : ", absent");
}

void configure(const Settings& s) {
  g_cal = s.batCal;
  BatteryMonitor::Config c = g_mon.config();
  c.lowMv = s.batLow;
  c.critMv = s.batCrit;
  c.critHystMv = BAT_CRIT_HYST_MV;
  c.lowHystMv = BAT_LOW_HYST_MV;
  c.critConfirm = BAT_CRIT_CONFIRM;
  c.presentMinMv = BAT_PRESENT_MIN_MV;
  g_mon.configure(c);  // keeps the latches
  g_emptyMv = s.batEmpty;
  g_fullMv = s.batFull;
  // Show the new calibration at once; thresholds are evaluated on the next sample.
  if (g_avg.count() > 0) {
    g_mv = batteryMvFromAdc(g_avg.mean(), BAT_DIV_R_TOP, BAT_DIV_R_BOTTOM, g_cal);
    g_pct = g_mon.present() ? batteryPercent(g_mv, g_emptyMv, g_fullMv) : 0;
  }
}

void tick(uint32_t nowMs) {
  if ((int32_t)(nowMs - g_nextMs) < 0) return;
  g_nextMs = nowMs + BAT_SAMPLE_MS;
  sample();
}

uint32_t mv() { return g_mv; }
uint32_t adcMv() { return g_adcMv; }
uint8_t pct() { return g_pct; }
bool low() { return g_mon.low(); }
bool crit() { return g_mon.crit(); }
bool present() { return g_mon.present(); }

}  // namespace Battery
