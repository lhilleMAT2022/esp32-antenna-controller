#pragma once

#include <stdint.h>

namespace antenna_controller {
// Poll update() even while no packets arrive, so expiry can trigger a redraw.
struct ReceiveFreshness {
    uint32_t lastReceivedMs = 0;
    bool receivedAny = false;
    bool wasFresh = false;

    void receive(uint32_t nowMs) {
        lastReceivedMs = nowMs;
        receivedAny = true;
    }

    bool fresh(uint32_t nowMs, uint32_t timeoutMs) const {
        return receivedAny && uint32_t(nowMs - lastReceivedMs) < timeoutMs;
    }

    bool update(uint32_t nowMs, uint32_t timeoutMs) {
        const bool nowFresh = fresh(nowMs, timeoutMs);
        const bool changed = nowFresh != wasFresh;
        wasFresh = nowFresh;
        // An expired timestamp must not become fresh after another millis wrap.
        if (!nowFresh) receivedAny = false;
        return changed;
    }
};
}  // namespace antenna_controller
