// Compile-time configuration of the turbine controller.
//
// This is the ONLY file in the project that contains GPIO numbers. Everything
// a user may want to tune at runtime also exists as a setting (see
// lib/turbine_core/src/settings_model.h); the values here are the defaults.
#pragma once

#include <stdint.h>

#if !defined(NATIVE_TEST) && defined(__has_include)
#if __has_include(<sdkconfig.h>)
#include <sdkconfig.h>  // defines CONFIG_IDF_TARGET_*
#endif
#endif

#define FW_VERSION "1.0.0"
#define PROTOCOL_VERSION 1

// ---------------------------------------------------------------------------
// Feature flags (override with -D in platformio.ini)
// ---------------------------------------------------------------------------
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1  // AP + web panel + WebSocket + HTTP API
#endif
#ifndef ENABLE_BLE
#define ENABLE_BLE 1  // NimBLE GATT service
#endif
#ifndef ENABLE_OTA
#define ENABLE_OTA 1  // firmware / filesystem upload at /update
#endif
#ifndef ENABLE_CAPTIVE
#define ENABLE_CAPTIVE 1  // compiled in; switched on by the `captive` setting
#endif
#ifndef ENABLE_SERIAL_CONSOLE
#define ENABLE_SERIAL_CONSOLE 1  // r50 / yl / yr / s ... commands on Serial
#endif
#if !ENABLE_WIFI
#undef ENABLE_OTA
#define ENABLE_OTA 0
#undef ENABLE_CAPTIVE
#define ENABLE_CAPTIVE 0
#endif

// ---------------------------------------------------------------------------
// Yaw position sensor: change this one line to switch modes
// ---------------------------------------------------------------------------
#define YAW_SENSOR_NONE 0      // no sensor, time-based twist protection
#define YAW_SENSOR_ENDSTOPS 1  // two limit switches, active LOW
#define YAW_SENSOR_AS5600 2    // magnetic absolute encoder on I2C
#ifndef YAW_SENSOR_MODE
#define YAW_SENSOR_MODE YAW_SENSOR_NONE
#endif

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ESP32-S3 (proposal, confirm when the board arrives; GPIO 35-37 are taken
// by octal PSRAM on N*R8 modules, 0/3/45/46 are strapping pins)
#define BOARD_NAME "ESP32-S3"
#define PIN_ROTOR_IN1 4
#define PIN_ROTOR_IN2 5
#define PIN_YAW_IN1 6
#define PIN_YAW_IN2 7
#define PIN_DRV_SLEEP 15
#define PIN_BAT_ADC 1  // ADC1 only (GPIO 1-10)
#define PIN_ENDSTOP_L 16
#define PIN_ENDSTOP_R 17
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define PIN_LED 48  // WS2812 on DevKitC-1 v1.0 (v1.1 boards use GPIO 38)
#define LED_IS_RGB 1
#define LED_ACTIVE_HIGH 1

#elif defined(CONFIG_IDF_TARGET_ESP32)
// ESP32 DevKit V1 (ESP32-WROOM-32D, 30 pin). Avoid 0, 2, 5, 12, 15 for
// inputs. GPIO 14 toggles during boot; harmless because SLEEP is LOW then.
#define BOARD_NAME "ESP32"
#define PIN_ROTOR_IN1 25
#define PIN_ROTOR_IN2 26
#define PIN_YAW_IN1 27
#define PIN_YAW_IN2 14
#define PIN_DRV_SLEEP 33
#define PIN_BAT_ADC 34  // ADC1, input only
#define PIN_ENDSTOP_L 16
#define PIN_ENDSTOP_R 17
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_LED 2  // on-board blue LED
#define LED_IS_RGB 0
#define LED_ACTIVE_HIGH 1

#elif defined(NATIVE_TEST)
// Host unit tests: pins are never touched.
#define BOARD_NAME "native"
#define PIN_ROTOR_IN1 0
#define PIN_ROTOR_IN2 0
#define PIN_YAW_IN1 0
#define PIN_YAW_IN2 0
#define PIN_DRV_SLEEP 0
#define PIN_BAT_ADC 0
#define PIN_ENDSTOP_L 0
#define PIN_ENDSTOP_R 0
#define PIN_I2C_SDA 0
#define PIN_I2C_SCL 0
#define PIN_LED 0
#define LED_IS_RGB 0
#define LED_ACTIVE_HIGH 1

#else
#error "Unsupported target: add a pin block to include/config.h"
#endif

// ---------------------------------------------------------------------------
// Motors (DRV8833: two PWM inputs per motor, 00 = coast, 11 = brake)
// ---------------------------------------------------------------------------
#define MOTOR_PWM_FREQ_HZ 20000  // above hearing, no whine
#define MOTOR_PWM_BITS 8
#define MOTOR_TICK_MS 10  // ramp update period
// 0 = fast decay (PWM on one input, other LOW). 1 = slow decay (one input
// HIGH, inverted PWM on the other): more linear speed vs duty on DC motors.
#ifndef MOTOR_SLOW_DECAY
#define MOTOR_SLOW_DECAY 0
#endif
// Hardware PWM ceilings (0-255). Motors are fed from 5 V: a bare 3 V motor
// should stay around 60 %, an N20 rated 3-9 V can take 100 %.
#define ROTOR_MAX_DUTY 153
#define YAW_MAX_DUTY 255
// Rotor ramp: full 0-100 % range in this time (spec: RAMP_STEP_PER_MS).
#define RAMP_FULL_SCALE_MS 500
#define RAMP_STEP_PER_MS (100.0f / RAMP_FULL_SCALE_MS)
#define YAW_RAMP_MS 120  // short soft start for the yaw gear motor
#define FS_RAMP_MS 200   // fixed rotor ramp-down used by failsafe trips
#define DRIVER_SLEEP_IDLE_MS 2000  // DRV8833 SLEEP LOW after both motors idle this long

// ---------------------------------------------------------------------------
// Yaw
// ---------------------------------------------------------------------------
#define YAW_SPEED_PCT 60
#define YAW_SPEED_MIN_PCT 20
#define YAW_SPEED_MAX_PCT 100
#define YAW_MAX_BALANCE_MS 0      // twist limit (speed-weighted ms), 0 = off
#define YAW_HOLD_TIMEOUT_MS 700   // deadman: stop if no `hold` for this long
#define YAW_BAL_SAVE_MS 1000      // persist balance at most this often while moving
#define YAW_CAL_MIN_MS 500        // shortest accepted ms-per-360 calibration
#define ENDSTOP_DEBOUNCE_MS 20
#define AS5600_I2C_ADDR 0x36
#define AS5600_PERIOD_MS 50
#define AS5600_I2C_TIMEOUT_MS 10
#define YAW_GOTO_WINDOW_DEG 2.0f
#define YAW_GOTO_SLOW_DEG 10.0f
#define YAW_GOTO_SLOW_PCT 30
#define YAW_GOTO_TIMEOUT_MS 10000

// ---------------------------------------------------------------------------
// Battery (divider 100k / 33k into ADC1)
// ---------------------------------------------------------------------------
#define BAT_DIV_R_TOP 100000.0f
#define BAT_DIV_R_BOTTOM 33000.0f
#define BAT_CAL 1.0f           // multiply factor, fix with a multimeter
#define BAT_SAMPLE_MS 1000
#define BAT_AVG_SAMPLES 8      // moving average length
#define BAT_OVERSAMPLE 8       // ADC reads averaged into one sample
#define BAT_CRIT_HYST_MV 300   // crit releases at crit + 0.3 V
#define BAT_LOW_HYST_MV 100
#define BAT_CRIT_CONFIRM 2     // consecutive samples below crit before latching
#define BAT_PRESENT_MIN_MV 2500  // below this the battery is treated as absent

#define BAT_PRESET_9V 0
#define BAT_PRESET_2S 1
#define BAT_PRESET_CUSTOM 2
#ifndef BAT_PRESET
#define BAT_PRESET BAT_PRESET_9V
#endif
#define BAT9V_LOW_MV 7000
#define BAT9V_CRIT_MV 6200
#define BAT9V_EMPTY_MV 6200
#define BAT9V_FULL_MV 9400
#define BAT2S_LOW_MV 6600
#define BAT2S_CRIT_MV 6000
#define BAT2S_EMPTY_MV 6000
#define BAT2S_FULL_MV 8400

// ---------------------------------------------------------------------------
// Failsafe
// ---------------------------------------------------------------------------
#define FS_TIMEOUT_MS 500      // silence from the motion owner before stopping
#define FS_TIMEOUT_MIN_MS 400  // must stay above the 300 ms client ping period
#define FS_TIMEOUT_MAX_MS 1000 // keeps "WiFi lost -> motors stop" under 1 s
#define FS_CHECK_MS 50
#define ROTOR_MAX_RUN_S 0      // 0 = no limit

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
#define AP_SSID_PREFIX "Turbina-"  // + 4 hex digits of the MAC
#define AP_PASSWORD "turbina123"   // min. 8 characters (WPA2)
#define AP_CHANNEL 1
#define AP_MAX_CLIENTS 2
#define AP_IP_STR "192.168.4.1"
#define WIFI_TX_QDBM 34            // 0.25 dBm units: 34 = 8.5 dBm
#define HTTP_PORT 80
#define OTA_USER "admin"           // password = AP password
#define WS_SILENT_CLOSE_MS 3000    // close WS clients that sent nothing this long
#define BLE_MODE_OFF 0
#define BLE_MODE_AUTO 1            // advertise only while no WiFi station is associated
#define BLE_MODE_ALWAYS 2
#define BLE_MODE_DEFAULT BLE_MODE_AUTO
#define BLE_SERVICE_UUID "a8d40001-6c7e-4f2b-9a3e-1b2c3d4e5f60"
#define BLE_CMD_UUID "a8d40002-6c7e-4f2b-9a3e-1b2c3d4e5f60"    // write / write without response
#define BLE_STATE_UUID "a8d40003-6c7e-4f2b-9a3e-1b2c3d4e5f60"  // notify
#define BLE_PREFERRED_MTU 247
#define BLE_TX_POWER_DBM 3

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
#define TELEMETRY_PERIOD_MS 200
#define TELEMETRY_MIN_GAP_MS 50     // "immediately on change", at most 20 frames/s
#define BLE_TELEMETRY_PERIOD_MS 200
#define BLE_TELEMETRY_MIN_GAP_MS 100
#define BLE_STATE_MAX_BYTES 180

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------
#define WDT_TIMEOUT_S 5
#define SERIAL_BAUD 115200
#define CMD_QUEUE_DEPTH 16
#define EVENT_LOG_SIZE 20
#define SETTINGS_SAVE_DELAY_MS 300  // coalesce slider-driven cfg writes
#define LOOP_WAIT_MS 2              // loop sleeps on the command queue at most this long
