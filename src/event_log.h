// Ring buffer of the last EVENT_LOG_SIZE notable events (failsafe trips and
// clears, battery warnings, sensor errors, boot / reset reason, OTA). Each
// add() also prints to Serial via LOG. The loop broadcasts new items to WS
// clients as "evt" frames. Loop task only.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "frames.h"

namespace EventLog {
void begin();
// lvl: 'e', 'w' or 'i'. code: short snake_case id (<= 15 chars).
void add(char lvl, const char* code, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
size_t snapshot(LogItem* out, size_t max);  // oldest first
bool popPending(LogItem& out);              // next item not yet broadcast
uint32_t revision();                        // increments on every add()
}  // namespace EventLog
