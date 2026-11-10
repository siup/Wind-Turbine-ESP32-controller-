// Ring buffer of the last EVENT_LOG_SIZE events. Items are indexed by a
// monotonic sequence number: slot = seq % EVENT_LOG_SIZE. The pending cursor
// is the sequence number of the next item the loop has not broadcast yet.

#include <Arduino.h>

#include "event_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "log.h"

namespace {

LogItem g_items[EVENT_LOG_SIZE];
uint32_t g_total = 0;    // items ever added (sequence number of the next one)
uint32_t g_pending = 0;  // sequence number of the next item to broadcast
uint32_t g_rev = 0;

char normalizeLevel(char lvl) {
  switch (lvl) {
    case 'e':
    case 'E':
      return 'e';
    case 'w':
    case 'W':
      return 'w';
    default:
      return 'i';
  }
}

// Bounded copy that always NUL-terminates and zero-fills the tail.
void copyTrunc(char* dst, size_t cap, const char* src) {
  size_t i = 0;
  if (src) {
    for (; i + 1 < cap && src[i]; i++) dst[i] = src[i];
  }
  for (; i < cap; i++) dst[i] = '\0';
}

// Number of items currently held.
uint32_t held() { return g_total < EVENT_LOG_SIZE ? g_total : (uint32_t)EVENT_LOG_SIZE; }

}  // namespace

namespace EventLog {

void begin() {
  memset(g_items, 0, sizeof(g_items));
  g_total = 0;
  g_pending = 0;
  g_rev = 0;
}

void add(char lvl, const char* code, const char* fmt, ...) {
  // Format the full text first: Serial gets all of it, the item a truncated copy.
  char full[128];
  full[0] = '\0';
  if (fmt) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(full, sizeof(full), fmt, ap);
    va_end(ap);
  }

  LogItem& it = g_items[g_total % EVENT_LOG_SIZE];
  it.ms = millis();
  it.lvl = normalizeLevel(lvl);
  copyTrunc(it.code, sizeof(it.code), code);
  copyTrunc(it.msg, sizeof(it.msg), full);

  g_total++;
  g_rev++;
  // Items overwritten before they were broadcast are skipped.
  if (g_total - g_pending > EVENT_LOG_SIZE) g_pending = g_total - EVENT_LOG_SIZE;

  const int level = it.lvl == 'e' ? LOG_ERROR : (it.lvl == 'w' ? LOG_WARN : LOG_INFO);
  LOG(level, "%s: %s", it.code, full);
}

size_t snapshot(LogItem* out, size_t max) {
  if (!out || max == 0) return 0;
  uint32_t n = held();
  if (n > max) n = (uint32_t)max;  // keep the newest n
  const uint32_t first = g_total - n;
  for (uint32_t i = 0; i < n; i++) out[i] = g_items[(first + i) % EVENT_LOG_SIZE];
  return n;
}

bool popPending(LogItem& out) {
  if (g_pending == g_total) return false;
  out = g_items[g_pending % EVENT_LOG_SIZE];
  g_pending++;
  return true;
}

uint32_t revision() { return g_rev; }

}  // namespace EventLog
