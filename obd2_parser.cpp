#include "obd2_parser.h"
#include "obd2_pids.h"
#include <iostream>
#include <iomanip>

namespace OBD2 {

void parseAndPrintResponse(uint32_t can_id, const struct can_frame& frame) {
    // Check if this is a successful response (0x41) to a request for current data (0x01)
    if (frame.data[1] == RESPONSE_CURRENT_DATA) {
        uint8_t pid = frame.data[2];
        uint8_t a = frame.data[3];
        uint8_t b = frame.data[4];

        std::cout << std::dec << "[" << std::hex << can_id << std::dec << "] ";

        switch(pid) {
            case PID_ENGINE_LOAD: 
                std::cout << "📈 Engine loading: " << (a * 100 / 255) << " %"; 
                break;
            case PID_COOLANT_TEMP: 
                std::cout << "🌡️ antifreeze: " << (a - 40) << " °C"; 
                break;
            case PID_MAP: 
                std::cout << "🌪️ Absolute pressure (MAP): " << (int)a << " kpa"; 
                break;
            case PID_RPM: 
                std::cout << "🔥 RPM: " << (((a * 256) + b) / 4) << " rpm"; 
                break;
            case PID_SPEED: 
                std::cout << "🚀 Speed: " << (int)a << " km/h"; 
                break;
            case PID_MAF: 
                std::cout << "💨 Air consumption: " << (((a * 256) + b) / 100.0) << " g/s"; 
                break;
            case PID_THROTTLE: 
                std::cout << "🦋 Throttle: " << (a * 100 / 255) << " %"; 
                break;
            case PID_OIL_TEMP: 
                std::cout << "🛢️ Oil temperature: " << (a - 40) << " °C"; 
                break;
            default:   
                std::cout << "❓ another PID: " << std::hex << (int)pid; 
                break;
        }
        std::cout << std::endl;
    }
}

} // namespace OBD2
