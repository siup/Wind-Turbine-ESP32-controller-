// Composition root: setup() brings the modules up in a safe order, loop()
// dispatches inbound messages, ticks every module and schedules telemetry.
// Everything here runs in the Arduino loop task (core 1); other tasks only
// talk to it through CmdQueue and the atomics in shared_state.h.

#include <Arduino.h>

#include <esp_mac.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "battery.h"
#include "cmd_parser.h"
#include "cmd_queue.h"
#include "commands.h"
#include "config.h"
#include "event_log.h"
#include "failsafe.h"
#include "frames.h"
#include "log.h"
#include "motors.h"
#include "ota.h"
#include "serial_console.h"
#include "settings.h"
#include "shared_state.h"
#include "status_led.h"
#include "transport_ble.h"
#include "transport_wifi.h"
#include "yaw.h"

namespace {

// ---------------------------------------------------------------------------
// Reply sinks, one per transport
// ---------------------------------------------------------------------------

class WsSink : public ReplySink {
 public:
  explicit WsSink(uint32_t client) : client_(client) {}
  void reply(const char* data, size_t len) override { TransportWifi::sendTo(client_, data, len); }

 private:
  uint32_t client_;
};

// BLE: errors and pongs always, acks only for frames that carried an "id".
// Replies go to the central that wrote the command, not to every peer.
class BleSink : public ReplySink {
 public:
  explicit BleSink(uint32_t handle) : handle_((uint16_t)handle) {}
  void reply(const char* data, size_t len) override { TransportBle::sendTo(handle_, data, len); }
  bool wantsAck(bool hasId) const override { return hasId; }
  size_t maxLen() const override { return TransportBle::maxFrameLen(handle_); }

 private:
  uint16_t handle_;
};

// HTTP: the request handler already answered; execution results show up in
// the telemetry.
class HttpSink : public ReplySink {
 public:
  void reply(const char* data, size_t len) override {
    (void)data;
    (void)len;
  }
  bool wantsAck(bool hasId) const override {
    (void)hasId;
    return false;
  }
  bool wantsReplies() const override { return false; }
};

class SerialSink : public ReplySink {
 public:
  void reply(const char* data, size_t len) override { SerialConsole::print(data, len); }
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

char g_deviceName[sizeof(Settings::name) + 1] = "";
const char* g_resetReason = "unknown";
bool g_settingsApplied = false;

bool g_rebootPending = false;
uint32_t g_rebootAtMs = 0;

InMsg g_msg;  // large (kMaxCmdLen + header), kept off the loop stack

// Telemetry scheduler
uint32_t g_telCheckMs = 0;
uint32_t g_wsSentMs = 0;
uint32_t g_wsSig = 0;
uint32_t g_stateCacheMs = 0;
uint32_t g_bleSentMs = 0;
uint32_t g_bleSig = 0;
bool g_telFirst = true;

// HTTP caches
uint32_t g_cfgRevPublished = 0;
uint32_t g_logRevPublished = 0;
uint32_t g_infoPublishedMs = 0;
bool g_cachesFirst = true;

constexpr uint32_t kTelCheckMs = 10;
constexpr uint32_t kInfoCacheMs = 5000;

char g_telBuf[768];
char g_bleBuf[256];
char g_frameBuf[2304];  // cfg / info / log cache frames (a full log is ~2.2 kB), evt frames

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default: return "unknown";
  }
}

bool resetIsAbnormal(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
    case ESP_RST_PWR_GLITCH:
    case ESP_RST_CPU_LOCKUP:
      return true;
    default:
      return false;
  }
}

// From the factory MAC (eFuse), so it works before WiFi starts.
void computeDeviceName(const Settings& s) {
  if (s.name[0]) {
    snprintf(g_deviceName, sizeof(g_deviceName), "%s", s.name);
    return;
  }
  uint8_t mac[6] = {};
  esp_efuse_mac_get_default(mac);
  snprintf(g_deviceName, sizeof(g_deviceName), AP_SSID_PREFIX "%02X%02X", mac[4], mac[5]);
}

void clientLabel(proto::Source src, uint32_t client, char* out, size_t cap) {
  snprintf(out, cap, "%s#%lu", proto::sourceName(src), (unsigned long)client);
}

void startWatchdog() {
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000u;
  cfg.idle_core_mask = 0;  // only the loop task is supervised
  cfg.trigger_panic = true;
  // Core 3.x starts with the TWDT already initialized (CONFIG_ESP_TASK_WDT_INIT).
  esp_err_t err = esp_task_wdt_reconfigure(&cfg);
  if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_init(&cfg);
  if (err != ESP_OK) LOGE("wdt: config failed (%d)", (int)err);
  err = esp_task_wdt_add(nullptr);
  if (err != ESP_OK) LOGE("wdt: add loop task failed (%d)", (int)err);
}

// A motion command that was queued together with a stop must not run after
// it (stop jumps to the front of the queue).
bool isMotionFrame(const InMsg& m) {
  const proto::Cmd t = peekCommandType(m.data, m.len);
  return t == proto::Cmd::Rotor || t == proto::Cmd::Yaw;
}

void dispatch(const InMsg& m, bool afterEstop) {
  switch (m.kind) {
    case MsgKind::Command: {
      if (afterEstop && isMotionFrame(m)) {
        char who[24];
        clientLabel(m.src, m.client, who, sizeof(who));
        LOGW("stop: dropped queued motion frame from %s", who);
        return;
      }
      switch (m.src) {
        case proto::Source::Ws: {
          WsSink sink(m.client);
          Commands::handle(m, sink);
          break;
        }
        case proto::Source::Ble: {
          BleSink sink(m.client);
          Commands::handle(m, sink);
          break;
        }
        case proto::Source::Http: {
          HttpSink sink;
          Commands::handle(m, sink);
          break;
        }
        default: {  // Serial console
          SerialSink sink;
          Commands::handle(m, sink);
          break;
        }
      }
      break;
    }
    case MsgKind::Connected: {
      char who[24];
      clientLabel(m.src, m.client, who, sizeof(who));
      LOGI("client %s connected", who);
      SerialConsole::disableKeepAlive();
      if (m.src == proto::Source::Ws) {
        WsSink sink(m.client);
        Commands::sendHello(sink);
      }
      break;
    }
    case MsgKind::Disconnected: {
      char who[24];
      clientLabel(m.src, m.client, who, sizeof(who));
      LOGI("client %s disconnected", who);
      Failsafe::onClientDisconnected(fs::ClientRef{m.src, m.client}, millis());
      break;
    }
  }
}

void processQueue() {
  bool afterEstop = false;
  if (shared::estopRequested.exchange(false)) {
    Failsafe::emergencyStop("stop request");
    afterEstop = true;
  }
  if (!CmdQueue::receive(g_msg, LOOP_WAIT_MS)) return;
  // Drain what is queued, bounded so a flood cannot starve the ticks.
  for (int i = 0; i <= CMD_QUEUE_DEPTH; i++) {
    if (shared::estopRequested.exchange(false)) {
      Failsafe::emergencyStop("stop request");
      afterEstop = true;
    }
    dispatch(g_msg, afterEstop);
    if (!CmdQueue::receive(g_msg, 0)) break;
  }
}

// With nobody connected (WS + BLE) the LED shows Idle even while a
// disconnect / timeout failsafe is latched: that is the normal state after
// the last phone left, not an alarm. Ota and BatteryCrit always show.
StatusLed::Mode ledMode() {
  if (shared::otaActive.load() || Ota::active()) return StatusLed::Mode::Ota;
  if (Battery::crit()) return StatusLed::Mode::BatteryCrit;
  if (TransportWifi::wsClients() + TransportBle::clientCount() == 0) return StatusLed::Mode::Idle;
  if (Failsafe::active()) return StatusLed::Mode::Failsafe;
  return StatusLed::Mode::Connected;
}

// FNV-1a over the brief BLE frame: changes exactly when something a BLE
// client can see changes.
uint32_t hashBytes(const char* p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) {
    h ^= (uint8_t)p[i];
    h *= 16777619u;
  }
  return h;
}

void telemetry(uint32_t now) {
  if (!g_telFirst && (uint32_t)(now - g_telCheckMs) < kTelCheckMs) return;
  g_telCheckMs = now;

  // WebSocket: full frame every TELEMETRY_PERIOD_MS, sooner on change (min gap
  // TELEMETRY_MIN_GAP_MS). The HTTP state cache is refreshed at least as often.
  const uint8_t ws = TransportWifi::wsClients();
  const uint32_t sinceWs = now - g_wsSentMs;
  const bool cacheDue = g_telFirst || (uint32_t)(now - g_stateCacheMs) >= TELEMETRY_PERIOD_MS;
  const bool wsMaySend = ws > 0 && sinceWs >= TELEMETRY_MIN_GAP_MS;
  if (wsMaySend || cacheDue) {
    uint32_t sig = 0;
    const size_t n = Commands::buildTelemetry(g_telBuf, sizeof(g_telBuf), false, &sig);
    if (n) {
      const bool send = wsMaySend && (sinceWs >= TELEMETRY_PERIOD_MS || sig != g_wsSig);
      if (send) {
        TransportWifi::broadcast(g_telBuf, n);
        g_wsSentMs = now;
        g_wsSig = sig;
      }
      if (send || cacheDue) {
        TransportWifi::publish(TransportWifi::Cache::State, g_telBuf, n);
        g_stateCacheMs = now;
      }
    }
  }

  // BLE: brief frame, its own period, gap and change tracking.
  if (TransportBle::clientCount() > 0) {
    const uint32_t sinceBle = now - g_bleSentMs;
    if (sinceBle >= BLE_TELEMETRY_MIN_GAP_MS) {
      const size_t n = Commands::buildTelemetry(g_bleBuf, sizeof(g_bleBuf), true, nullptr);
      if (n) {
        const uint32_t sig = hashBytes(g_bleBuf, n);
        if (sinceBle >= BLE_TELEMETRY_PERIOD_MS || sig != g_bleSig) {
          TransportBle::notify(g_bleBuf, n);
          g_bleSentMs = now;
          g_bleSig = sig;
        }
      }
    }
  }
  g_telFirst = false;
}

void refreshCaches(uint32_t now) {
  const uint32_t cfgRev = SettingsStore::revision();
  if (g_cachesFirst || cfgRev != g_cfgRevPublished) {
    const size_t n = Commands::buildCfgFrame(g_frameBuf, sizeof(g_frameBuf));
    if (n) TransportWifi::publish(TransportWifi::Cache::Cfg, g_frameBuf, n);
    g_cfgRevPublished = cfgRev;
  }
  if (g_cachesFirst || (uint32_t)(now - g_infoPublishedMs) >= kInfoCacheMs) {
    const size_t n = Commands::buildInfoFrame(g_frameBuf, sizeof(g_frameBuf));
    if (n) TransportWifi::publish(TransportWifi::Cache::Info, g_frameBuf, n);
    g_infoPublishedMs = now;
  }
  const uint32_t logRev = EventLog::revision();
  if (g_cachesFirst || logRev != g_logRevPublished) {
    const size_t n = Commands::buildLogFrame(g_frameBuf, sizeof(g_frameBuf));
    if (n) TransportWifi::publish(TransportWifi::Cache::Log, g_frameBuf, n);
    g_logRevPublished = logRev;
  }
  g_cachesFirst = false;
}

// New event log items go to WS clients as "evt" frames.
void broadcastEvents() {
  const bool anyWs = TransportWifi::wsClients() > 0;
  LogItem item;
  while (EventLog::popPending(item)) {
    if (!anyWs) continue;
    const size_t n = buildEvt(item, g_frameBuf, sizeof(g_frameBuf));
    if (n) TransportWifi::broadcast(g_frameBuf, n);
  }
}

void serviceReboot(uint32_t now) {
  if (!g_rebootPending || (int32_t)(now - g_rebootAtMs) < 0) return;
  g_rebootPending = false;
  LOGI("restarting");
  yaw.saveNow();
  SettingsStore::flush();
  Serial.flush();
  ESP.restart();
}

}  // namespace

// ---------------------------------------------------------------------------
// app.h
// ---------------------------------------------------------------------------

void appApplySettings(const Settings& prev, const Settings& next) {
  const bool boot = !g_settingsApplied;
  g_settingsApplied = true;

  MotorDriver::Config rc;
  rc.maxDuty = next.rotorDuty;
  rc.minPct = next.rotorMin;
  rc.rampMs = next.rotorRamp;
  rc.invert = next.rotorRev;
  rc.slowDecay = MOTOR_SLOW_DECAY != 0;
  rotorMotor.configure(rc);
  // rotorMax is the ceiling of the running speed too, not only of new
  // commands: a lower value ramps a faster rotor down to it. Ownership does
  // not change (no onMotionCommand). A failsafe ramp or a brake has target 0.
  if (!boot && rotorMotor.target() > (int8_t)next.rotorMax) rotorMotor.setSpeed((int8_t)next.rotorMax);

  MotorDriver::Config yc;
  yc.maxDuty = next.yawDuty;
  yc.minPct = 0;
  yc.rampMs = YAW_RAMP_MS;
  yc.invert = next.yawInvert;
  yc.slowDecay = MOTOR_SLOW_DECAY != 0;
  yawMotor.configure(yc);

  yaw.configure(prev, next);
  Battery::configure(next);
  Failsafe::configure(next);
  if (boot || prev.txPower != next.txPower) TransportWifi::setTxPower(next.txPower);
  TransportBle::configure(next);
  StatusLed::configure(next);
  // The OTA password follows the AP password the network runs with, which
  // changes only with a restart: TransportWifi::begin() sets it.
}

void appRequestReboot(uint32_t delayMs, const char* why) {
  Failsafe::emergencyStop("reboot");
  EventLog::add('i', "reboot", "%s, in %lu ms", (why && why[0]) ? why : "requested", (unsigned long)delayMs);
  g_rebootPending = true;
  g_rebootAtMs = millis() + delayMs;
}

void appBroadcast(const char* data, size_t len, bool toBle) {
  TransportWifi::broadcast(data, len);
  if (toBle) TransportBle::notify(data, len);
}

const char* appDeviceName() {
  if (!g_deviceName[0]) computeDeviceName(SettingsStore::get());
  return g_deviceName;
}

const char* appResetReason() { return g_resetReason; }

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  motorsEarlyInit();  // first: motor pins LOW before anything else runs

#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // never block on a USB host that is not reading
#else
  Serial.setTxBufferSize(1024);  // logging goes to a buffer, not to the wire
#endif
  Serial.begin(SERIAL_BAUD);

  const esp_reset_reason_t rr = esp_reset_reason();
  g_resetReason = resetReasonName(rr);
  EventLog::begin();
  EventLog::add(resetIsAbnormal(rr) ? 'w' : 'i', "boot", "fw %s, reset %s", FW_VERSION, g_resetReason);

  SettingsStore::begin();
  const Settings& s = SettingsStore::get();
  computeDeviceName(s);
  LOGI("device %s, board %s", g_deviceName, BOARD_NAME);

  DriverPower::begin();
  rotorMotor.begin(PIN_ROTOR_IN1, PIN_ROTOR_IN2, "rotor");
  yawMotor.begin(PIN_YAW_IN1, PIN_YAW_IN2, "yaw");
  Battery::begin();
  yaw.begin();
  Failsafe::begin();
  appApplySettings(s, s);

  CmdQueue::begin();
  TransportWifi::begin(s);
  TransportBle::begin(s);
  StatusLed::begin();
  SerialConsole::begin();
  Commands::begin();

  startWatchdog();
  LOGI("ready, heap %lu", (unsigned long)ESP.getFreeHeap());
}

void loop() {
  // (1)+(2) estop flag, then commands (wakes at once when a frame arrives)
  processQueue();

  // (3) module ticks
  const uint32_t now = millis();
  Failsafe::tick(now);
  rotorMotor.tick(now);
  yaw.tick(now);
  yawMotor.tick(now);
  DriverPower::tick(now, !rotorMotor.idle() || !yawMotor.idle());
  Battery::tick(now);
  SettingsStore::tick(now);
  TransportWifi::tick(now);
  TransportBle::tick(now, TransportWifi::stations());
  Ota::tick(now);
  SerialConsole::tick(now);
  StatusLed::tick(now, ledMode());

  // (4) telemetry, (5) HTTP caches, (6) live events
  telemetry(now);
  refreshCaches(now);
  broadcastEvents();

  // (7) deferred restart
  serviceReboot(now);

  // (8)
  esp_task_wdt_reset();
}
