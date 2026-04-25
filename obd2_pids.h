#pragma once

#include <cstdint>

namespace OBD2 {

    // CAN IDs
    constexpr uint32_t CAN_ID_GATEWAY = 0x17F00010;
    constexpr uint32_t CAN_ID_ECU = 0x7E8;
    constexpr uint32_t CAN_ID_GEARBOX = 0x7E9;

    // Service / Modes
    constexpr uint8_t SERVICE_CURRENT_DATA = 0x01;
    constexpr uint8_t RESPONSE_CURRENT_DATA = 0x41;

    // PIDs
    constexpr uint8_t PID_ENGINE_LOAD = 0x04;
    constexpr uint8_t PID_COOLANT_TEMP = 0x05;
    constexpr uint8_t PID_MAP = 0x0B;
    constexpr uint8_t PID_RPM = 0x0C;
    constexpr uint8_t PID_SPEED = 0x0D;
    constexpr uint8_t PID_MAF = 0x10;
    constexpr uint8_t PID_THROTTLE = 0x11;
    constexpr uint8_t PID_OIL_TEMP = 0x5C;

} // namespace OBD2
