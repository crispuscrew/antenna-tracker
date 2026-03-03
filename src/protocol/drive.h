#pragma once
#include <cstdint>

// Current antenna state parsed from the controller's 30-byte response.
struct Drive {
    uint16_t seq   = 0;
    uint8_t  state = 0;   // raw 4-char state field, first nibble used
    float    az    = 0.f; // degrees, may exceed [0,360) due to unwrap
    float    el    = 0.f; // degrees [0, 90]
};

// Commands sent to the controller.
enum class DriveCmd : int {
    TRACKER = 2,  // 0x02 — active tracking
    STOP    = 3,  // 0x03 — stop/park
};
