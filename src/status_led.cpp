// Status LED patterns, non-blocking. Each mode is a periodic on/off pattern
// whose phase restarts when the mode changes, so a failsafe trip always
// starts with a visible flash. The pin (or the WS2812 colour) is written
// only when the output actually changes.

#include "status_led.h"

#include <Arduino.h>

#include "config.h"

namespace {

using StatusLed::Mode;

// Output colour. A single LED only looks at "lit" (any channel non-zero).
struct Rgb {
  uint8_t r, g, b;
  bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
  bool operator!=(const Rgb& o) const { return !(*this == o); }
  bool lit() const { return r || g || b; }
};

// Low brightness (<= 32 per channel): a WS2812 at full power is blinding
// and pulls ~50 mA from the 5 V rail the motors share.
constexpr Rgb kDark{0, 0, 0};
constexpr Rgb kWhite{12, 12, 12};
constexpr Rgb kGreen{0, 24, 0};
constexpr Rgb kAmber{28, 10, 0};
constexpr Rgb kRed{32, 0, 0};
constexpr Rgb kBlue{0, 0, 32};

bool g_enabled = true;
bool g_begun = false;
Mode g_mode = Mode::Boot;
uint32_t g_modeStartMs = 0;
bool g_written = false;  // false until the first write (forces it)
Rgb g_last = kDark;

void writeOut(const Rgb& c) {
  if (g_written && c == g_last) return;
  g_written = true;
  g_last = c;
#if LED_IS_RGB
  rgbLedWrite(PIN_LED, c.r, c.g, c.b);
#else
  const bool on = c.lit();
  digitalWrite(PIN_LED, (on == (LED_ACTIVE_HIGH != 0)) ? HIGH : LOW);
#endif
}

// True while the pattern of mode m is in an "on" window at phase t (ms
// since the mode started).
bool patternOn(Mode m, uint32_t t) {
  switch (m) {
    case Mode::Boot:
      return true;  // solid
    case Mode::Idle:
      return (t % 1000u) < 100u;  // 1 Hz, short flash
    case Mode::Connected: {
      // Heartbeat, mostly on: two short dips every 1.2 s.
      const uint32_t p = t % 1200u;
      return !(p < 60u || (p >= 180u && p < 240u));
    }
    case Mode::Failsafe:
      return (t % 250u) < 125u;  // 4 Hz
    case Mode::BatteryCrit: {
      // Double blink every second.
      const uint32_t p = t % 1000u;
      return p < 100u || (p >= 200u && p < 300u);
    }
    case Mode::Ota:
      return (t % 100u) < 50u;  // 10 Hz flicker
    case Mode::Off:
    default:
      return false;
  }
}

Rgb modeColour(Mode m) {
  switch (m) {
    case Mode::Boot: return kWhite;
    case Mode::Idle: return kGreen;
    case Mode::Connected: return kGreen;
    case Mode::Failsafe: return kAmber;
    case Mode::BatteryCrit: return kRed;
    case Mode::Ota: return kBlue;
    case Mode::Off:
    default: return kDark;
  }
}

}  // namespace

namespace StatusLed {

void begin() {
#if !LED_IS_RGB
  pinMode(PIN_LED, OUTPUT);
#endif
  g_begun = true;
  g_mode = Mode::Boot;
  g_modeStartMs = millis();
  g_written = false;
  writeOut(g_enabled ? modeColour(Mode::Boot) : kDark);
}

// Called from appApplySettings(), possibly before begin(): only remember
// the flag here, tick() applies it.
void configure(const Settings& s) { g_enabled = s.led; }

void tick(uint32_t nowMs, Mode m) {
  if (!g_begun) return;
  if (!g_enabled) m = Mode::Off;
  if (m != g_mode) {
    g_mode = m;
    g_modeStartMs = nowMs;
  }
  const bool on = patternOn(m, nowMs - g_modeStartMs);
  writeOut(on ? modeColour(m) : kDark);
}

}  // namespace StatusLed
