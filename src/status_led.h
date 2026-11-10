// Status LED patterns (single LED on ESP32, WS2812 colours on S3 via
// rgbLedWrite). Non-blocking. Loop task only.
//   Boot         solid
//   Idle         1 Hz blink, short on        (AP up, nobody connected over
//                                             WS / BLE, even with a latched
//                                             disconnect / timeout failsafe)
//   Connected    heartbeat, mostly on         (WS / BLE client present)
//   Failsafe     4 Hz blink                   (amber on RGB; only while a
//                                             WS / BLE client is connected)
//   BatteryCrit  double blink every second    (red on RGB)
//   Ota          fast flicker                 (blue on RGB)
//   Off          dark (led setting false)
#pragma once

#include <stdint.h>

#include "settings_model.h"

namespace StatusLed {
enum class Mode : uint8_t { Boot, Idle, Connected, Failsafe, BatteryCrit, Ota, Off };
void begin();
void configure(const Settings& s);  // led on/off
void tick(uint32_t nowMs, Mode m);
}  // namespace StatusLed
