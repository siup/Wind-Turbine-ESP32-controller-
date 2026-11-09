// Leveled logging to Serial: LOG(LOG_WARN, "battery %u mV", mv);
// Output: "[    12345] W battery 6100 mV"
#pragma once

#include <Arduino.h>

#define LOG_ERROR 1
#define LOG_WARN 2
#define LOG_INFO 3
#define LOG_DEBUG 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

#define LOG(level, fmt, ...)                                                   \
  do {                                                                         \
    if ((level) <= LOG_LEVEL) {                                                \
      Serial.printf("[%9lu] %c " fmt "\n", (unsigned long)millis(),            \
                    "?EWID"[(level) & 7], ##__VA_ARGS__);                      \
    }                                                                          \
  } while (0)

#define LOGE(fmt, ...) LOG(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG(LOG_WARN, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG(LOG_INFO, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) LOG(LOG_DEBUG, fmt, ##__VA_ARGS__)
