#pragma once

#include <linux/can.h>
#include <cstdint>

namespace OBD2 {
    void parseAndPrintResponse(uint32_t can_id, const struct can_frame& frame);
}
