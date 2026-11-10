// Serial console (stage 1 bench tests and debugging). Lines are turned into
// protocol frames and pushed to CmdQueue with Source::Console:
//   r<0-100>   rotor speed            ({"t":"rotor","speed":N})
//   rb         rotor stop with brake
//   yl / yr    yaw left / right (deadman fed by `ka` or repeated lines)
//   ys         yaw stop
//   s          stop everything
//   g<deg>     yaw goto (AS5600)
//   b          print battery (mV, raw ADC mV, %, flags)
//   tel on|off print the short state every 500 ms
//   ka on|off  keepalive: ping + yaw hold every 200 ms so the failsafe
//              and the yaw deadman do not trip during bench tests. RAM only,
//              switched off automatically when a WS or BLE client connects.
//   {...}      any JSON frame as-is
//   ?          help
// Replies are printed as JSON lines. Loop task only.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace SerialConsole {
void begin();
void tick(uint32_t nowMs);
void print(const char* data, size_t len);  // one reply line
void disableKeepAlive();                    // called when a WS / BLE client connects
}  // namespace SerialConsole
