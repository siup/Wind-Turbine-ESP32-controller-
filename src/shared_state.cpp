// Definitions of the cross-task flags declared in shared_state.h.

#include "shared_state.h"

namespace shared {

std::atomic<bool> otaActive{false};
std::atomic<bool> motorsIdle{false};
std::atomic<bool> estopRequested{false};

}  // namespace shared
