// Command layer: parse an inbound frame, run it against the modules, answer
// through the ReplySink. Also gathers the telemetry snapshot and builds the
// cfg / info / log frames. Loop task only. Semantics: docs/PROTOCOL.md.

#include <Arduino.h>

#include "commands.h"

#include <esp_mac.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "battery.h"
#include "cmd_parser.h"
#include "config.h"
#include "event_log.h"
#include "failsafe.h"
#include "frames.h"
#include "log.h"
#include "motors.h"
#include "settings.h"
#include "settings_model.h"
#include "shared_state.h"
#include "transport_ble.h"
#include "transport_wifi.h"
#include "yaw.h"

using proto::Cmd;
using proto::Err;
using proto::YawAction;
using proto::YawDir;

namespace {

// Frame buffers (loop task only, one frame in flight at a time per buffer).
char g_replyBuf[192];  // ack / err / pong
char g_stateBuf[768];
char g_cfgBuf[1024];
char g_infoBuf[512];
char g_logBuf[2048];

LogItem g_logItems[EVENT_LOG_SIZE];

// Settings being patched (kept off the stack).
Settings g_prev;
Settings g_next;

char g_mac[18] = "";

const char* const kBuild = __DATE__ " " __TIME__;

// "ws#3", "ble#1", "http@192.168.4.2", "serial" (Source::Console).
void formatClient(const InMsg& m, char* out, size_t cap) {
  if (m.src == proto::Source::Http) {
    snprintf(out, cap, "http@%u.%u.%u.%u", (unsigned)(m.client & 0xff), (unsigned)((m.client >> 8) & 0xff),
             (unsigned)((m.client >> 16) & 0xff), (unsigned)((m.client >> 24) & 0xff));
  } else if (m.src == proto::Source::Ws || m.src == proto::Source::Ble) {
    snprintf(out, cap, "%s#%lu", proto::sourceName(m.src), (unsigned long)m.client);
  } else {
    snprintf(out, cap, "%s", proto::sourceName(m.src));
  }
}

void sendErr(ReplySink& sink, const Command& c, Err e, const char* field = nullptr) {
  if (!sink.wantsReplies()) return;
  const size_t n = buildErr(c.type, e, field, nullptr, c.hasId, c.id, g_replyBuf, sizeof(g_replyBuf));
  if (n) sink.reply(g_replyBuf, n);
}

void sendAck(ReplySink& sink, const Command& c) {
  if (!sink.wantsReplies() || !sink.wantsAck(c.hasId)) return;
  const size_t n = buildAck(c.type, c.hasId, c.id, g_replyBuf, sizeof(g_replyBuf));
  if (n) sink.reply(g_replyBuf, n);
}

void sendPong(ReplySink& sink, const Command& c) {
  if (!sink.wantsReplies()) return;
  const size_t n = buildPong(c.hasId, c.id, millis(), g_replyBuf, sizeof(g_replyBuf));
  if (n) sink.reply(g_replyBuf, n);
}

void replyFrame(ReplySink& sink, const char* buf, size_t n) {
  if (n && sink.wantsReplies()) sink.reply(buf, n);
}

bool motorsIdleNow() { return rotorMotor.idle() && yaw.idle(); }

// Index of the first kFieldIdleOnly field that differs between a and b, or -1.
int idleOnlyChange(const Settings& a, const Settings& b) {
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    if ((kSettingsFields[i].flags & kFieldIdleOnly) && settingsFieldDiffers(a, b, i)) return (int)i;
  }
  return -1;
}

bool anyChange(const Settings& a, const Settings& b) {
  for (size_t i = 0; i < kSettingsFieldCount; i++) {
    if (settingsFieldDiffers(a, b, i)) return true;
  }
  return false;
}

// Push the new cfg frame to every WS client (two phones stay in sync).
void broadcastCfg() {
  const size_t n = Commands::buildCfgFrame(g_cfgBuf, sizeof(g_cfgBuf));
  if (n) appBroadcast(g_cfgBuf, n, false);
}

// g_prev holds the settings before the change, g_next the new ones.
void commitNext() {
  if (anyChange(g_prev, g_next)) {
    SettingsStore::commit(g_next);
    appApplySettings(g_prev, g_next);
  }
}

// ---------------------------------------------------------------------------
// Per-command handlers
// ---------------------------------------------------------------------------

void doRotor(const InMsg& m, const Command& c, ReplySink& sink, uint32_t now) {
  const fs::ClientRef who{m.src, m.client};
  if (c.speed > 0) {
    const Err e = Failsafe::allowMotion();
    if (e != Err::None) return sendErr(sink, c, e);
    Failsafe::clearRuntime();
    uint8_t sp = c.speed;
    const uint8_t maxPct = SettingsStore::get().rotorMax;
    if (sp > maxPct) sp = maxPct;
    rotorMotor.setSpeed((int8_t)sp);
    Failsafe::onMotionCommand(who, now);
  } else if (c.brake) {
    rotorMotor.brake();
  } else {
    rotorMotor.setSpeed(0);
  }
  sendAck(sink, c);
}

void doYaw(const InMsg& m, const Command& c, ReplySink& sink, uint32_t now) {
  const fs::ClientRef who{m.src, m.client};
  Err e = Err::None;
  switch (c.yawAction) {
    case YawAction::Move:
      if (c.dir == YawDir::Stop) {
        yaw.stop(true);
        break;
      }
      e = Failsafe::allowMotion();
      if (e == Err::None) e = yaw.move(c.dir, c.yawSpeed, now);
      if (e == Err::None) Failsafe::onMotionCommand(who, now);
      break;
    case YawAction::Hold:
      yaw.hold(c.dir, now);
      break;
    case YawAction::Goto:
      e = Failsafe::allowMotion();
      if (e == Err::None) e = yaw.gotoAngle(c.gotoDeg, now);
      if (e == Err::None) Failsafe::onMotionCommand(who, now);
      break;
    case YawAction::Zero:
      e = yaw.zero();
      break;
    case YawAction::CalStart:
    case YawAction::CalCancel:
      e = yaw.calibrate(c.yawAction, nullptr);
      break;
    case YawAction::CalStop: {
      uint32_t ms = 0;
      e = yaw.calibrate(YawAction::CalStop, &ms);
      if (e != Err::None) break;
      g_prev = SettingsStore::get();
      g_next = g_prev;
      const FieldDesc* f = settingsFindField("yawMs360");
      if (!f || !settingsSetNumber(g_next, *f, (double)ms)) {
        e = Err::OutOfRange;
        break;
      }
      commitNext();
      sendAck(sink, c);
      broadcastCfg();
      return;
    }
  }
  if (e != Err::None) return sendErr(sink, c, e);
  sendAck(sink, c);
}

void doStop(const InMsg& m, const Command& c, ReplySink& sink) {
  // The estop flag set by CmdQueue has normally stopped everything already
  // at the top of this loop pass; stop again only if something still moves
  // (keeps the event log to one entry per press).
  if (!motorsIdleNow() || rotorMotor.braking()) {
    char who[28];
    formatClient(m, who, sizeof(who));
    Failsafe::emergencyStop(who);
  }
  sendAck(sink, c);
}

void doCfg(const Command& c, ReplySink& sink) {
  if (c.cfgGet) {
    replyFrame(sink, g_cfgBuf, Commands::buildCfgFrame(g_cfgBuf, sizeof(g_cfgBuf)));
    return;
  }

  g_prev = SettingsStore::get();
  if (c.cfgReset) {
    settingsDefaults(g_next);
  }
  // cfg-set: g_next was filled by parseCommand (whole patch valid).

  const int busy = idleOnlyChange(g_prev, g_next);
  if (busy >= 0 && !motorsIdleNow()) return sendErr(sink, c, Err::Busy, kSettingsFields[busy].key);

  if (c.cfgReset) {
    SettingsStore::factoryReset();
    appApplySettings(g_prev, SettingsStore::get());
  } else {
    commitNext();
  }
  sendAck(sink, c);
  broadcastCfg();
}

bool otaAllows(Cmd t) { return t == Cmd::Stop || t == Cmd::Ping || t == Cmd::Info || t == Cmd::Log; }

void fillSnapshot(TelemetrySnapshot& t) {
  const int8_t tgt = rotorMotor.target();
  const int8_t act = rotorMotor.actual();
  t.rotorTarget = tgt > 0 ? (uint8_t)tgt : 0;
  t.rotorActual = act > 0 ? (uint8_t)act : 0;
  t.rotorRunS = Failsafe::rotorRunS();

  yaw.fill(t);

  t.batMv = Battery::mv();
  t.batPct = Battery::pct();
  t.batLow = Battery::low();
  t.batCrit = Battery::crit();
  t.batPresent = Battery::present();

  t.ws = TransportWifi::wsClients();
  t.sta = TransportWifi::stations();
  t.ble = TransportBle::clientCount();
  int8_t r = 0;
  if (t.sta > 0 && TransportWifi::rssi(r)) {
    t.hasRssi = true;
    t.rssi = r;
  } else if (t.ble > 0 && TransportBle::rssi(r)) {
    t.hasRssi = true;
    t.rssi = r;
  }

  t.fsActive = Failsafe::active();
  t.fsReason = Failsafe::reason();
  t.ota = shared::otaActive.load();
  t.cfgRev = SettingsStore::revision();
  t.upMs = millis();
}

const char* macString() {
  if (!g_mac[0]) {
    uint8_t mac[6] = {};
    esp_efuse_mac_get_default(mac);
    snprintf(g_mac, sizeof(g_mac), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4],
             mac[5]);
  }
  return g_mac;
}

}  // namespace

namespace Commands {

void begin() { macString(); }

void handle(const InMsg& msg, ReplySink& sink) {
  if (msg.kind != MsgKind::Command) return;
  const uint32_t now = millis();

  Command c;
  // cfg-set is patched against the current settings into g_next (atomic:
  // g_next is only written when the whole patch is valid).
  const ParseResult pr = parseCommand(msg.data, msg.len, c, &SettingsStore::get(), &g_next);
  if (pr.err != Err::None) {
    sendErr(sink, c, pr.err, pr.field);
    return;
  }

  Failsafe::onFrame(fs::ClientRef{msg.src, msg.client}, now);

  if (shared::otaActive.load() && !otaAllows(c.type)) {
    sendErr(sink, c, Err::BusyOta);
    return;
  }

  switch (c.type) {
    case Cmd::Ping:
      sendPong(sink, c);
      break;
    case Cmd::Stop:
      doStop(msg, c, sink);
      break;
    case Cmd::Rotor:
      doRotor(msg, c, sink, now);
      break;
    case Cmd::Yaw:
      doYaw(msg, c, sink, now);
      break;
    case Cmd::Cfg:
      doCfg(c, sink);
      break;
    case Cmd::Info:
      replyFrame(sink, g_infoBuf, buildInfoFrame(g_infoBuf, sizeof(g_infoBuf)));
      break;
    case Cmd::Log:
      replyFrame(sink, g_logBuf, buildLogFrame(g_logBuf, sizeof(g_logBuf)));
      break;
    case Cmd::Reboot:
      sendAck(sink, c);
      appRequestReboot(300, "reboot command");
      break;
    case Cmd::None:
      sendErr(sink, c, Err::UnknownCmd);
      break;
  }
}

void sendHello(ReplySink& sink) {
  replyFrame(sink, g_infoBuf, buildInfoFrame(g_infoBuf, sizeof(g_infoBuf)));
  replyFrame(sink, g_cfgBuf, buildCfgFrame(g_cfgBuf, sizeof(g_cfgBuf)));
  replyFrame(sink, g_stateBuf, buildTelemetry(g_stateBuf, sizeof(g_stateBuf), false, nullptr));
}

size_t buildTelemetry(char* out, size_t cap, bool brief, uint32_t* signatureOut) {
  TelemetrySnapshot t;
  fillSnapshot(t);
  if (signatureOut) *signatureOut = stateSignature(t);
  return buildState(t, brief, out, cap);
}

size_t buildCfgFrame(char* out, size_t cap) {
  return buildCfg(SettingsStore::get(), SettingsStore::revision(), SettingsStore::rebootPending(), out, cap);
}

size_t buildInfoFrame(char* out, size_t cap) {
  DeviceInfo d;
  d.fw = FW_VERSION;
  d.build = kBuild;
  d.name = appDeviceName();
  d.chip = BOARD_NAME;
  d.mac = macString();
  d.sensor = yaw.sensorName();
  d.resetReason = appResetReason();
  d.wifi = ENABLE_WIFI;
  d.ble = ENABLE_BLE;
  d.ota = ENABLE_OTA;
  d.heap = ESP.getFreeHeap();
  d.heapMin = ESP.getMinFreeHeap();
  d.upMs = millis();
  return buildInfo(d, out, cap);
}

size_t buildLogFrame(char* out, size_t cap) {
  const size_t n = EventLog::snapshot(g_logItems, EVENT_LOG_SIZE);
  // Drop the oldest items until the frame fits.
  for (size_t first = 0;; first++) {
    const size_t len = buildLog(g_logItems + first, n - first, out, cap);
    if (len || first >= n) return len;
  }
}

}  // namespace Commands
