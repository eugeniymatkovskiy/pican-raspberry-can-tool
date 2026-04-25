#pragma once

// ISO 15765-2 (ISO-TP) layer implemented directly on top of SocketCAN raw.
// Reference: ISO 15765-2:2016 (Road vehicles -- Diagnostic communication over
// Controller Area Network -- Part 2: Transport protocol and network layer
// services).
//
// PCI (Protocol Control Information) byte 0 high nibble:
//   0x0  Single Frame   (SF)  -- low nibble = payload length (0..7)
//   0x1  First Frame    (FF)  -- 12-bit total length across byte0.low + byte1
//   0x2  Consecutive Fr (CF)  -- low nibble = sequence counter (1..F wrapping)
//   0x3  Flow Control   (FC)  -- low nibble = status (0 CTS, 1 Wait, 2 Overflow)
//
// This implementation targets "normal addressing" (no N_AI byte), which is what
// VW/MQB diagnostics use. Extended (29-bit) CAN IDs are supported via the
// `extended_id` flag in the Config.

#include <cstdint>
#include <cstddef>
#include <vector>

namespace isotp {

enum class Status {
    Ok,
    TimeoutRx,          // No CAN frame received within timeout_ms
    TimeoutFc,          // No Flow Control received after First Frame
    SocketError,        // write()/read() failed
    UnexpectedPci,      // Got a PCI we didn't expect for this state
    SequenceError,      // CF sequence counter mismatch
    OverflowFc,         // Receiver told us its buffer overflowed
    TooLong,            // Payload > 4095 bytes
    BadFrame,           // Short read, garbage data, etc.
};

const char* statusToString(Status s);

struct Config {
    // CAN identifiers, already stripped of the CAN_EFF_FLAG.
    uint32_t tx_id         = 0x18DA10F1;   // To Gateway (ECU 19)
    uint32_t rx_id         = 0x18DAF110;   // From Gateway (ECU 19)
    bool     extended_id   = true;         // 29-bit addressing

    // Padding used on outgoing frames shorter than 8 bytes.
    // VW commonly uses 0xAA (and accepts 0xCC); 0xAA is safest.
    uint8_t  padding       = 0xAA;

    // Values the *receiver* (i.e. we) advertise in the Flow Control frame
    // that we send after receiving a First Frame.
    uint8_t  rx_block_size = 0x00;         // 0 = send everything at once
    uint8_t  rx_st_min     = 0x00;         // 0 = as fast as possible

    // Overall transport-layer timeouts.
    int      timeout_ms    = 1500;

    // Verbose hex logging of every CAN frame to std::cout.
    bool     verbose       = true;
};

class IsoTp {
public:
    IsoTp(int socket_fd, const Config& cfg);

    // Send a full UDS service PDU. Handles SF or FF+CFs automatically and
    // waits for Flow Control frames when required.
    Status send(const std::vector<uint8_t>& payload);

    // Receive a full UDS service PDU. Handles SF or FF+CFs automatically and
    // emits Flow Control back to the ECU when required.
    Status recv(std::vector<uint8_t>& payload);

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

private:
    int    fd_;
    Config cfg_;

    // Raw 8-byte CAN frame I/O.
    bool writeCanFrame(const uint8_t data[8]);
    // Returns:
    //   1  = frame received into `out` (always 8 bytes padded)
    //   0  = timeout
    //  -1  = socket error
    int  readCanFrame(uint8_t out[8], int timeout_ms);

    void logFrame(const char* dir, uint32_t id, const uint8_t data[8]);
};

} // namespace isotp
