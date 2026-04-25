#pragma once

// UDS (ISO 14229-1) client over ISO-TP / SocketCAN.
//
// All byte sequences follow the UDS request/response model:
//
//   Positive response:   SID+0x40   [...data...]
//   Negative response:   0x7F  SID  NRC
//
// NRC 0x78 ("requestCorrectlyReceived-ResponsePending") is handled internally
// by re-arming the RX timer and waiting for the real response.

#include "iso_tp.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace uds {

// --- UDS Service IDs ---
namespace SID {
constexpr uint8_t DIAGNOSTIC_SESSION_CONTROL = 0x10;
constexpr uint8_t ECU_RESET                  = 0x11;
constexpr uint8_t SECURITY_ACCESS            = 0x27;
constexpr uint8_t COMMUNICATION_CONTROL      = 0x28;
constexpr uint8_t TESTER_PRESENT             = 0x3E;
constexpr uint8_t READ_DATA_BY_IDENTIFIER    = 0x22;
constexpr uint8_t WRITE_DATA_BY_IDENTIFIER   = 0x2E;
constexpr uint8_t ROUTINE_CONTROL            = 0x31;
} // namespace SID

// --- Diagnostic Session Types (SID 0x10) ---
namespace Session {
constexpr uint8_t DEFAULT      = 0x01;
constexpr uint8_t PROGRAMMING  = 0x02;
constexpr uint8_t EXTENDED     = 0x03;   // <-- needed for adaptations
constexpr uint8_t SAFETY       = 0x04;
} // namespace Session

// --- Security Access sub-function pairs ---
// VW Gateways commonly use levels 0x03 / 0x04 (or 0x0B / 0x0C, 0x11 / 0x12)
// for adaptation-related writes. The odd value requests the seed; the even
// value (odd+1) submits the key.
namespace SecLevel {
constexpr uint8_t REQ_SEED_L1 = 0x01;  constexpr uint8_t SEND_KEY_L1 = 0x02;
constexpr uint8_t REQ_SEED_L3 = 0x03;  constexpr uint8_t SEND_KEY_L3 = 0x04;
constexpr uint8_t REQ_SEED_L5 = 0x05;  constexpr uint8_t SEND_KEY_L5 = 0x06;
constexpr uint8_t REQ_SEED_L7 = 0x07;  constexpr uint8_t SEND_KEY_L7 = 0x08;
} // namespace SecLevel

// Negative Response Codes (subset used for friendly error messages).
const char* nrcToString(uint8_t nrc);

// Unified result type for a UDS transaction.
struct Response {
    bool                  ok        = false;
    uint8_t               sid       = 0;    // echoed (positive) or 0x7F (negative)
    uint8_t               nrc       = 0;    // valid only when ok == false and transport succeeded
    isotp::Status         tp_status = isotp::Status::Ok;
    std::vector<uint8_t>  data;             // everything after SID on a positive response

    std::string describe() const;
};

class UdsClient {
public:
    UdsClient(int socket_fd, const isotp::Config& cfg);

    // Low-level: send a raw UDS request (starting with the SID byte) and
    // return whatever came back. Handles NRC 0x78 "response pending".
    Response request(const std::vector<uint8_t>& req,
                     int response_timeout_ms = 2000,
                     int max_response_pending = 10);

    // ---- High-level helpers ----
    Response diagnosticSessionControl(uint8_t session);
    Response testerPresent(bool suppress_pos_response = true);

    Response readDataByIdentifier(uint16_t did);
    Response writeDataByIdentifier(uint16_t did, const std::vector<uint8_t>& data);

    // Security Access with a pluggable seed->key function.
    // `req_seed_sub` must be odd (e.g. 0x03); key is sent with req_seed_sub+1.
    // If the ECU responds with a zero-length seed, that means "already unlocked"
    // and the function returns success without calling `key_fn`.
    using KeyFn = std::function<std::vector<uint8_t>(const std::vector<uint8_t>& seed)>;
    Response securityAccess(uint8_t req_seed_sub, KeyFn key_fn);

    // Accessors.
    isotp::IsoTp&       tp()       { return tp_; }
    const isotp::IsoTp& tp() const { return tp_; }

private:
    isotp::IsoTp tp_;
    bool verbose_;

    void logReq(const std::vector<uint8_t>& req);
    void logResp(const Response& r);
};

} // namespace uds
