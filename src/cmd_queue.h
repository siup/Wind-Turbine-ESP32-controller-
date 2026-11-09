// Inbound message queue: transports (any task) -> loop task.
// The loop blocks on receive() with a short timeout, so a command wakes it
// immediately and an idle loop does not burn CPU.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

enum class MsgKind : uint8_t {
  Command,       // data holds one JSON frame
  Connected,     // a WS / BLE client connected (loop sends hello: info + cfg + state)
  Disconnected,  // a WS / BLE client went away (failsafe owner check)
};

struct InMsg {
  MsgKind kind = MsgKind::Command;
  proto::Source src = proto::Source::None;
  uint32_t client = 0;  // WS client id, BLE conn handle, HTTP remote IPv4, 0 for Serial
  uint16_t len = 0;
  char data[proto::kMaxCmdLen + 1] = {};  // NUL-terminated
};

namespace CmdQueue {

void begin();  // creates the FreeRTOS queue (CMD_QUEUE_DEPTH items)

// Any task, never blocks. Copies the frame. A {"t":"stop"} frame (detected
// with peekCommandType) raises shared::estopRequested and goes to the FRONT
// of the queue. Returns false if len > kMaxCmdLen or the queue is full; the
// caller should then answer err too_long / busy itself.
bool pushCommand(proto::Source src, uint32_t client, const char* data, size_t len);

// Any task, never blocks. Connect / disconnect notifications. Disconnected
// is pushed to the front so the failsafe sees it before stale commands.
bool pushEvent(MsgKind kind, proto::Source src, uint32_t client);

// Loop task only. Waits up to timeoutMs for a message.
bool receive(InMsg& out, uint32_t timeoutMs);

// Messages rejected because the queue was full (diagnostics).
uint32_t dropped();

}  // namespace CmdQueue
