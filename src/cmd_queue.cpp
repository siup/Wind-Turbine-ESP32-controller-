// Inbound message queue: transports (any task) -> loop task.
//
// A statically allocated FreeRTOS queue of InMsg. Producers never block
// (timeout 0). {"t":"stop"} raises shared::estopRequested before anything
// else and jumps to the front of the queue. Connect / disconnect events that
// do not fit into a full queue go to a tiny overflow list that receive()
// drains first, so the failsafe never loses a disconnect because a client
// flooded the queue.
#include "cmd_queue.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>

#include <atomic>

#include "cmd_parser.h"
#include "config.h"
#include "shared_state.h"

namespace {

QueueHandle_t s_queue = nullptr;
StaticQueue_t s_queueCtrl;
uint8_t s_queueStorage[CMD_QUEUE_DEPTH * sizeof(InMsg)];

std::atomic<uint32_t> s_dropped{0};

// Overflow for MsgKind::Connected / Disconnected (FIFO, portMUX protected).
struct PendingEvent {
  MsgKind kind;
  proto::Source src;
  uint32_t client;
};
constexpr uint8_t kOverflowSize = 8;
PendingEvent s_overflow[kOverflowSize];
uint8_t s_ovHead = 0;   // index of the oldest entry
uint8_t s_ovCount = 0;  // entries in use
std::atomic<uint8_t> s_ovPending{0};  // lock-free "is there anything" check
portMUX_TYPE s_ovMux = portMUX_INITIALIZER_UNLOCKED;

bool overflowPush(MsgKind kind, proto::Source src, uint32_t client) {
  bool ok = false;
  portENTER_CRITICAL(&s_ovMux);
  if (s_ovCount < kOverflowSize) {
    PendingEvent& e = s_overflow[(s_ovHead + s_ovCount) % kOverflowSize];
    e.kind = kind;
    e.src = src;
    e.client = client;
    s_ovCount++;
    s_ovPending.store(s_ovCount);
    ok = true;
  }
  portEXIT_CRITICAL(&s_ovMux);
  return ok;
}

bool overflowPop(InMsg& out) {
  bool ok = false;
  PendingEvent e{};
  portENTER_CRITICAL(&s_ovMux);
  if (s_ovCount > 0) {
    e = s_overflow[s_ovHead];
    s_ovHead = (s_ovHead + 1) % kOverflowSize;
    s_ovCount--;
    s_ovPending.store(s_ovCount);
    ok = true;
  }
  portEXIT_CRITICAL(&s_ovMux);
  if (ok) {
    out.kind = e.kind;
    out.src = e.src;
    out.client = e.client;
    out.len = 0;
    out.data[0] = '\0';
  }
  return ok;
}

}  // namespace

namespace CmdQueue {

void begin() {
  if (s_queue) return;
  s_queue = xQueueCreateStatic(CMD_QUEUE_DEPTH, sizeof(InMsg), s_queueStorage, &s_queueCtrl);
}

bool pushCommand(proto::Source src, uint32_t client, const char* data, size_t len) {
  if (!data || len > proto::kMaxCmdLen) return false;

  // Detect stop before touching the queue: the flag works even when the
  // queue is full or not created yet.
  const bool isStop = peekCommandType(data, len) == proto::Cmd::Stop;
  if (isStop) shared::estopRequested.store(true);

  if (!s_queue) {
    s_dropped.fetch_add(1);
    return false;
  }

  InMsg msg;
  msg.kind = MsgKind::Command;
  msg.src = src;
  msg.client = client;
  msg.len = static_cast<uint16_t>(len);
  memcpy(msg.data, data, len);
  msg.data[len] = '\0';

  const BaseType_t ok = isStop ? xQueueSendToFront(s_queue, &msg, 0) : xQueueSendToBack(s_queue, &msg, 0);
  if (ok != pdTRUE) {
    s_dropped.fetch_add(1);
    return false;
  }
  return true;
}

bool pushEvent(MsgKind kind, proto::Source src, uint32_t client) {
  if (s_queue) {
    InMsg msg;
    msg.kind = kind;
    msg.src = src;
    msg.client = client;
    const BaseType_t ok = (kind == MsgKind::Disconnected) ? xQueueSendToFront(s_queue, &msg, 0)
                                                          : xQueueSendToBack(s_queue, &msg, 0);
    if (ok == pdTRUE) return true;
  }
  if (overflowPush(kind, src, client)) return true;
  s_dropped.fetch_add(1);
  return false;
}

bool receive(InMsg& out, uint32_t timeoutMs) {
  // Events that did not fit into the queue come first (they are older than
  // anything still waiting in it, and disconnects matter to the failsafe).
  if (s_ovPending.load() && overflowPop(out)) return true;
  if (!s_queue) return false;

  TickType_t ticks = 0;
  if (timeoutMs > 0) {
    ticks = pdMS_TO_TICKS(timeoutMs);
    if (ticks == 0) ticks = 1;
  }
  return xQueueReceive(s_queue, &out, ticks) == pdTRUE;
}

uint32_t dropped() { return s_dropped.load(); }

}  // namespace CmdQueue
