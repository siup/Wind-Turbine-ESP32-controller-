// BLE transport on NimBLE-Arduino 2.x.
//
// Threads: GATT / GAP callbacks run in the NimBLE host task. They only touch
// the peer table below (portMUX protected), the state-frame cache (same
// lock), a few atomics and CmdQueue::push*(). Everything else (advertising
// policy, RSSI, notify, logging) runs in the loop task. NimBLE API calls are
// never made while the spinlock is held, and callbacks do not printf: the
// host task has a 4 kB stack and its write path already holds a buffer of
// the attribute's max length.
//
// Stack start: in begin() when bleMode is not off. If the device boots with
// bleMode off and it is switched on later, the stack is started from tick()
// once all motors are idle (NimBLEDevice::init blocks until the host syncs).

#include "transport_ble.h"

#include "config.h"

#if ENABLE_BLE

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>

#include <atomic>

#include "app.h"
#include "cmd_parser.h"
#include "cmd_queue.h"
#include "frames.h"
#include "log.h"
#include "protocol.h"
#include "shared_state.h"

namespace {

constexpr uint8_t kMaxPeers = CONFIG_BT_NIMBLE_MAX_CONNECTIONS;
constexpr uint16_t kNoConn = BLE_HS_CONN_HANDLE_NONE;
constexpr uint16_t kDefaultMtu = 23;  // ATT default before the MTU exchange

// Connection parameters requested on connect: 15-30 ms interval (1.25 ms
// units), no slave latency, 4 s supervision timeout (10 ms units).
constexpr uint16_t kConnItvlMin = 12;
constexpr uint16_t kConnItvlMax = 24;
constexpr uint16_t kConnLatency = 0;
constexpr uint16_t kConnTimeout = 400;
constexpr uint16_t kDataLenOctets = 251;  // one LL packet for a whole state frame
// cmd attribute size: above kMaxCmdLen so an oversized frame still reaches
// onWrite and gets err too_long; anything longer fails at the ATT level.
constexpr uint16_t kCmdAttrMaxLen = proto::kMaxCmdLen + 64;

// Advertising interval 100-150 ms (0.625 ms units): quick to find, cheap.
constexpr uint16_t kAdvItvlMin = 160;
constexpr uint16_t kAdvItvlMax = 240;

constexpr uint32_t kAdvCheckMs = 100;   // advertising policy re-check
constexpr uint32_t kAdvRetryMs = 1000;  // after a failed start
constexpr uint32_t kRssiPeriodMs = 1000;

struct Peer {
  uint16_t handle = kNoConn;
  uint16_t mtu = kDefaultMtu;
  bool subscribed = false;  // notifications enabled on the state characteristic
  bool mtuWarned = false;   // "frame larger than MTU" logged for this connection
};

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Peer g_peers[kMaxPeers];  // g_mux
std::atomic<uint8_t> g_peerCount{0};
// Disconnect reasons are logged from tick(), not from the host task.
std::atomic<uint32_t> g_discCount{0};
std::atomic<int> g_discReason{0};
std::atomic<uint16_t> g_discHandle{0};

// Last brief state frame, returned by a READ of the state characteristic.
char g_stateCache[BLE_STATE_MAX_BYTES + 1];  // g_mux
size_t g_stateCacheLen = 0;                  // g_mux

// Loop task only
uint8_t g_mode = BLE_MODE_DEFAULT;
bool g_started = false;
bool g_initFailed = false;
NimBLEServer* g_server = nullptr;
NimBLECharacteristic* g_stateChr = nullptr;  // written once in startStack()
NimBLEAdvertising* g_adv = nullptr;
uint8_t g_lastStations = 0xff;
uint8_t g_lastMode = 0xff;
uint8_t g_lastPeers = 0xff;
uint32_t g_advCheckMs = 0;
uint32_t g_advRetryAtMs = 0;
bool g_advRetryPending = false;
uint32_t g_rssiMs = 0;
int8_t g_rssi = 0;
bool g_rssiValid = false;
uint32_t g_discLogged = 0;

// ---------------------------------------------------------------------------
// Peer table helpers (any task)
// ---------------------------------------------------------------------------

// Index of handle in g_peers, -1 if absent. Caller holds g_mux.
int findPeerLocked(uint16_t handle) {
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (g_peers[i].handle == handle) return i;
  }
  return -1;
}

uint8_t countPeersLocked() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (g_peers[i].handle != kNoConn) n++;
  }
  return n;
}

// Decide whether `len` bytes may go to the peer `handle` (subscribed and
// the frame fits its MTU). With wantWarn, *warn is set the first time per
// connection a frame is skipped because of the MTU.
bool mayNotifyPeer(uint16_t handle, size_t len, bool wantWarn, uint16_t* mtuOut, bool* warn) {
  bool ok = false;
  *warn = false;
  portENTER_CRITICAL(&g_mux);
  const int i = findPeerLocked(handle);
  if (i >= 0 && g_peers[i].subscribed) {
    const uint16_t mtu = g_peers[i].mtu;
    *mtuOut = mtu;
    if (mtu > 3 && len <= (size_t)(mtu - 3)) {
      ok = true;
    } else if (wantWarn && !g_peers[i].mtuWarned) {
      g_peers[i].mtuWarned = true;
      *warn = true;
    }
  }
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

// Notify one peer. Not under the lock: NimBLE takes its own. canLog only
// from the loop task.
void notifyPeer(uint16_t handle, const char* data, size_t len, bool canLog) {
  NimBLECharacteristic* chr = g_stateChr;
  if (!chr || !data || !len) return;
  uint16_t mtu = 0;
  bool warn = false;
  if (!mayNotifyPeer(handle, len, canLog, &mtu, &warn)) {
    if (warn) {
      LOGW("ble: conn %u MTU %u too small for a %u B frame, skipped", (unsigned)handle, (unsigned)mtu,
           (unsigned)len);
    }
    return;
  }
  chr->notify(reinterpret_cast<const uint8_t*>(data), len, handle);
}

// err frame straight back to the peer that wrote (host task).
void notifyError(uint16_t handle, proto::Cmd of, proto::Err e) {
  char buf[128];
  const size_t n = buildErr(of, e, nullptr, nullptr, false, 0, buf, sizeof(buf));
  if (n) notifyPeer(handle, buf, n, false);
}

bool isStateFrame(const char* data, size_t len) {
  static const char kPrefix[] = "{\"t\":\"state\"";
  return len >= sizeof(kPrefix) - 1 && memcmp(data, kPrefix, sizeof(kPrefix) - 1) == 0;
}

// ---------------------------------------------------------------------------
// NimBLE callbacks (host task)
// ---------------------------------------------------------------------------

class ServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    const uint16_t h = info.getConnHandle();
    const uint16_t mtu = info.getMTU();  // usually still the default here
    // The table has CONFIG_BT_NIMBLE_MAX_CONNECTIONS slots, the stack
    // never accepts more connections than that, so a slot is always free.
    portENTER_CRITICAL(&g_mux);
    int i = findPeerLocked(h);
    if (i < 0) i = findPeerLocked(kNoConn);
    if (i >= 0) {
      g_peers[i].handle = h;
      g_peers[i].mtu = mtu > kDefaultMtu ? mtu : kDefaultMtu;
      g_peers[i].subscribed = false;
      g_peers[i].mtuWarned = false;
    }
    g_peerCount.store(countPeersLocked());
    portEXIT_CRITICAL(&g_mux);

    server->updateConnParams(h, kConnItvlMin, kConnItvlMax, kConnLatency, kConnTimeout);
    server->setDataLen(h, kDataLenOctets);
    CmdQueue::pushEvent(MsgKind::Connected, proto::Source::Ble, h);
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    (void)server;
    const uint16_t h = info.getConnHandle();
    portENTER_CRITICAL(&g_mux);
    const int i = findPeerLocked(h);
    if (i >= 0) g_peers[i] = Peer{};
    g_peerCount.store(countPeersLocked());
    portEXIT_CRITICAL(&g_mux);
    g_discReason.store(reason);
    g_discHandle.store(h);
    g_discCount.fetch_add(1);
    // Advertising is restarted (or not) by tick().
    CmdQueue::pushEvent(MsgKind::Disconnected, proto::Source::Ble, h);
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo& info) override {
    const uint16_t h = info.getConnHandle();
    portENTER_CRITICAL(&g_mux);
    const int i = findPeerLocked(h);
    if (i >= 0) {
      g_peers[i].mtu = mtu;
      g_peers[i].mtuWarned = false;
    }
    portEXIT_CRITICAL(&g_mux);
  }
};

class CmdCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& info) override {
    const uint16_t h = info.getConnHandle();
    const NimBLEAttValue v = chr->getValue();
    const char* data = v.c_str();
    const size_t len = v.size();
    if (!data || len == 0) return;
    const proto::Cmd type = peekCommandType(data, len);
    if (len > proto::kMaxCmdLen) {
      notifyError(h, type, proto::Err::TooLong);
      return;
    }
    if (CmdQueue::pushCommand(proto::Source::Ble, h, data, len)) return;
    // A stop raised the estop flag even though the queue was full.
    if (type != proto::Cmd::Stop) notifyError(h, type, proto::Err::Busy);
  }
};

class StateCallbacks : public NimBLECharacteristicCallbacks {
 public:
  // Serve the latest state frame on a plain READ.
  void onRead(NimBLECharacteristic* chr, NimBLEConnInfo& info) override {
    (void)info;
    char buf[sizeof(g_stateCache)];
    size_t n = 0;
    portENTER_CRITICAL(&g_mux);
    n = g_stateCacheLen;
    memcpy(buf, g_stateCache, n);
    portEXIT_CRITICAL(&g_mux);
    chr->setValue(reinterpret_cast<const uint8_t*>(buf), n);
  }

  void onSubscribe(NimBLECharacteristic* chr, NimBLEConnInfo& info, uint16_t subValue) override {
    (void)chr;
    const uint16_t h = info.getConnHandle();
    portENTER_CRITICAL(&g_mux);
    const int i = findPeerLocked(h);
    if (i >= 0) g_peers[i].subscribed = (subValue & 0x0001) != 0;
    portEXIT_CRITICAL(&g_mux);
  }
};

ServerCallbacks g_serverCb;
CmdCallbacks g_cmdCb;
StateCallbacks g_stateCb;

// ---------------------------------------------------------------------------
// Loop task
// ---------------------------------------------------------------------------

bool startStack() {
  if (g_started) return true;
  if (g_initFailed) return false;
  const char* name = appDeviceName();
  const uint32_t t0 = millis();
  if (!NimBLEDevice::init(name)) {
    g_initFailed = true;
    LOGE("ble: NimBLE init failed, BLE disabled until restart");
    return false;
  }
  NimBLEDevice::setPower(BLE_TX_POWER_DBM);
  NimBLEDevice::setMTU(BLE_PREFERRED_MTU);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(&g_serverCb, false);
  g_server->advertiseOnDisconnect(false);  // tick() owns the advertising policy

  NimBLEService* svc = g_server->createService(BLE_SERVICE_UUID);
  NimBLECharacteristic* cmd =
      svc->createCharacteristic(BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR, kCmdAttrMaxLen);
  cmd->setCallbacks(&g_cmdCb);
  NimBLECharacteristic* state =
      svc->createCharacteristic(BLE_STATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 256);
  state->setCallbacks(&g_stateCb);
  g_stateChr = state;
  g_server->start();

  // Advertising packet: flags + complete 128-bit service UUID (21 of 31
  // bytes). The name goes into the scan response.
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.addServiceUUID(NimBLEUUID(BLE_SERVICE_UUID));
  NimBLEAdvertisementData scanData;
  scanData.setName(name);

  g_adv = NimBLEDevice::getAdvertising();
  g_adv->enableScanResponse(true);
  g_adv->setMinInterval(kAdvItvlMin);
  g_adv->setMaxInterval(kAdvItvlMax);
  if (!g_adv->setAdvertisementData(advData) || !g_adv->setScanResponseData(scanData)) {
    LOGE("ble: advertising data rejected");
  }

  g_started = true;
  g_lastMode = 0xff;  // force a policy evaluation
  LOGI("ble: up as %s in %lu ms, MTU %u", name, (unsigned long)(millis() - t0), (unsigned)BLE_PREFERRED_MTU);
  return true;
}

// Advertise while the mode wants it and a connection slot is free. Turning
// advertising off (WiFi station arrived, bleMode off) never drops a peer
// that is already connected; it stays until it leaves. Re-checked
// periodically too, because NimBLE restarts advertising by itself after a
// host reset.
void applyAdvertising(uint32_t nowMs, uint8_t stations) {
  const uint8_t peers = g_peerCount.load();
  const bool inputsChanged = stations != g_lastStations || g_mode != g_lastMode || peers != g_lastPeers;
  const bool periodic = (uint32_t)(nowMs - g_advCheckMs) >= kAdvCheckMs;
  if (!inputsChanged && !periodic) return;
  g_advCheckMs = nowMs;
  g_lastStations = stations;
  g_lastMode = g_mode;
  g_lastPeers = peers;

  const bool desired = g_mode == BLE_MODE_ALWAYS || (g_mode == BLE_MODE_AUTO && stations == 0);
  const bool want = desired && peers < kMaxPeers;
  const bool active = g_adv->isAdvertising();
  if (want == active) {
    g_advRetryPending = false;
    return;
  }
  if (!want) {
    g_adv->stop();
    g_advRetryPending = false;
    LOGD("ble: advertising off");
    return;
  }
  if (g_advRetryPending && (int32_t)(nowMs - g_advRetryAtMs) < 0) return;
  if (g_adv->start()) {
    g_advRetryPending = false;
    LOGD("ble: advertising on");
  } else {
    g_advRetryPending = true;
    g_advRetryAtMs = nowMs + kAdvRetryMs;
    LOGW("ble: advertising start failed, retry in %lu ms", (unsigned long)kAdvRetryMs);
  }
}

// First connected peer, kNoConn if none.
uint16_t firstPeer() {
  uint16_t h = kNoConn;
  portENTER_CRITICAL(&g_mux);
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (g_peers[i].handle != kNoConn) {
      h = g_peers[i].handle;
      break;
    }
  }
  portEXIT_CRITICAL(&g_mux);
  return h;
}

// ble_gap_conn_rssi() is a synchronous HCI round trip: cached, once a second.
void refreshRssi(uint32_t nowMs) {
  if ((uint32_t)(nowMs - g_rssiMs) < kRssiPeriodMs) return;
  g_rssiMs = nowMs;
  const uint16_t h = firstPeer();
  if (h == kNoConn) {
    g_rssiValid = false;
    return;
  }
  int8_t v = 0;
  if (ble_gap_conn_rssi(h, &v) == 0 && v != 127) {  // 127 = not available
    g_rssi = v;
    g_rssiValid = true;
  }
}

}  // namespace

namespace TransportBle {

void begin(const Settings& s) {
  g_mode = s.bleMode;
  if (g_mode == BLE_MODE_OFF) {
    LOGI("ble: off (bleMode 0), stack not started");
    return;
  }
  startStack();
}

// May run before begin() (appApplySettings at boot): only remember the mode.
void configure(const Settings& s) { g_mode = s.bleMode; }

void tick(uint32_t nowMs, uint8_t wifiStations) {
  if (!g_started) {
    // Switched on at runtime after booting with bleMode off. init() blocks
    // for the host sync, so wait until nothing is moving.
    if (g_mode == BLE_MODE_OFF || g_initFailed || !shared::motorsIdle.load()) return;
    if (!startStack()) return;
  }
  const uint32_t disc = g_discCount.load();
  if (disc != g_discLogged) {
    g_discLogged = disc;
    LOGI("ble: conn %u closed, reason 0x%03x", (unsigned)g_discHandle.load(), (unsigned)g_discReason.load());
  }
  applyAdvertising(nowMs, wifiStations);
  if (g_peerCount.load() > 0) {
    refreshRssi(nowMs);
  } else {
    g_rssiValid = false;
  }
}

void notify(const char* data, size_t len) {
  if (!g_started || !data || len == 0) return;

  if (isStateFrame(data, len) && len <= BLE_STATE_MAX_BYTES) {
    portENTER_CRITICAL(&g_mux);
    memcpy(g_stateCache, data, len);
    g_stateCacheLen = len;
    portEXIT_CRITICAL(&g_mux);
  }

  if (g_peerCount.load() == 0) return;
  uint16_t handles[kMaxPeers];
  uint8_t n = 0;
  portENTER_CRITICAL(&g_mux);
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (g_peers[i].handle != kNoConn) handles[n++] = g_peers[i].handle;
  }
  portEXIT_CRITICAL(&g_mux);
  for (uint8_t i = 0; i < n; i++) notifyPeer(handles[i], data, len, true);
}

void sendTo(uint16_t handle, const char* data, size_t len) {
  if (!g_started || !data || len == 0) return;
  notifyPeer(handle, data, len, true);
}

size_t maxFrameLen(uint16_t handle) {
  size_t n = 0;
  portENTER_CRITICAL(&g_mux);
  const int i = findPeerLocked(handle);
  if (i >= 0 && g_peers[i].mtu > 3) n = g_peers[i].mtu - 3;
  portEXIT_CRITICAL(&g_mux);
  return n;
}

bool hasPeer(uint16_t handle) {
  if (handle == kNoConn) return false;
  portENTER_CRITICAL(&g_mux);
  const bool found = findPeerLocked(handle) >= 0;
  portEXIT_CRITICAL(&g_mux);
  return found;
}

uint8_t clientCount() { return g_peerCount.load(); }

bool rssi(int8_t& out) {
  if (!g_rssiValid || g_peerCount.load() == 0) return false;
  out = g_rssi;
  return true;
}

}  // namespace TransportBle

#else  // !ENABLE_BLE

namespace TransportBle {
void begin(const Settings& s) { (void)s; }
void configure(const Settings& s) { (void)s; }
void tick(uint32_t nowMs, uint8_t wifiStations) {
  (void)nowMs;
  (void)wifiStations;
}
void notify(const char* data, size_t len) {
  (void)data;
  (void)len;
}
void sendTo(uint16_t handle, const char* data, size_t len) {
  (void)handle;
  (void)data;
  (void)len;
}
size_t maxFrameLen(uint16_t handle) {
  (void)handle;
  return 0;
}
bool hasPeer(uint16_t handle) {
  (void)handle;
  return false;
}
uint8_t clientCount() { return 0; }
bool rssi(int8_t& out) {
  (void)out;
  return false;
}
}  // namespace TransportBle

#endif  // ENABLE_BLE
