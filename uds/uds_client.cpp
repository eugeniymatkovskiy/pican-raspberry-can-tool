#include "uds_client.h"
#include "hex_utils.h"

#include <iostream>
#include <sstream>

namespace uds {

const char* nrcToString(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFunctionNotSupported";
        case 0x13: return "incorrectMessageLengthOrInvalidFormat";
        case 0x14: return "responseTooLong";
        case 0x21: return "busyRepeatRequest";
        case 0x22: return "conditionsNotCorrect";
        case 0x24: return "requestSequenceError";
        case 0x25: return "noResponseFromSubnetComponent";
        case 0x26: return "failurePreventsExecutionOfRequestedAction";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x35: return "invalidKey";
        case 0x36: return "exceededNumberOfAttempts";
        case 0x37: return "requiredTimeDelayNotExpired";
        case 0x70: return "uploadDownloadNotAccepted";
        case 0x71: return "transferDataSuspended";
        case 0x72: return "generalProgrammingFailure";
        case 0x73: return "wrongBlockSequenceCounter";
        case 0x78: return "requestCorrectlyReceived-ResponsePending";
        case 0x7E: return "subFunctionNotSupportedInActiveSession";
        case 0x7F: return "serviceNotSupportedInActiveSession";
        default:   return "unknown";
    }
}

std::string Response::describe() const {
    std::ostringstream os;
    if (tp_status != isotp::Status::Ok) {
        os << "Transport error: " << isotp::statusToString(tp_status);
        return os.str();
    }
    if (ok) {
        os << "OK  SID=0x" << std::hex << static_cast<int>(sid)
           << "  data=[" << hexu::bytesToHex(data) << "]";
    } else {
        os << "NACK  NRC=0x" << std::hex << static_cast<int>(nrc)
           << " (" << nrcToString(nrc) << ")";
    }
    return os.str();
}

UdsClient::UdsClient(int socket_fd, const isotp::Config& cfg)
    : tp_(socket_fd, cfg), verbose_(cfg.verbose) {}

void UdsClient::logReq(const std::vector<uint8_t>& req) {
    if (!verbose_) return;
    std::cout << "[UDS  TX] " << hexu::bytesToHex(req) << std::endl;
}

void UdsClient::logResp(const Response& r) {
    if (!verbose_) return;
    std::cout << "[UDS  RX] " << r.describe() << std::endl;
}

// ---------------------------------------------------------------------------
// request(): send a UDS PDU and return the first meaningful response.
//
// If you've used `fetch()`, this is the same idea:
//   - `req` is the HTTP-body equivalent (the raw UDS bytes).
//   - We block until the server answers, or give up on timeout.
//   - A "positive response" is HTTP 2xx: the first byte is req[0]+0x40.
//   - A "negative response" is HTTP 4xx/5xx: [0x7F, sid, NRC].
//   - NRC 0x78 is HTTP 102 "Processing" -- the ECU is saying "still thinking,
//     don't give up on me". We just keep reading until it either sends a
//     real answer or we've swallowed `max_response_pending` of these in a row.
//
// This function hides all of that so every caller sees a clean
// `Response { ok, sid, data, nrc }` record.
// ---------------------------------------------------------------------------
Response UdsClient::request(const std::vector<uint8_t>& req,
                            int response_timeout_ms,
                            int max_response_pending) {
    Response r{};
    if (req.empty()) { r.tp_status = isotp::Status::BadFrame; return r; }

    logReq(req);

    // Step 1: put the request on the bus (this may span many CAN frames
    // if the request itself is > 7 bytes, e.g. a 100-byte WDBI body).
    isotp::Status st = tp_.send(req);
    if (st != isotp::Status::Ok) { r.tp_status = st; logResp(r); return r; }

    // Step 2: while waiting for the *response* we usually want a longer
    // timeout than the default transport timeout, because the ECU might
    // take a moment to compute a key, write EEPROM, etc. We temporarily
    // swap in a bigger timeout and restore the old one at the end.
    auto cfg = tp_.config();
    int saved = cfg.timeout_ms;
    cfg.timeout_ms = response_timeout_ms;
    tp_.setConfig(cfg);

    int pending_count = 0;                 // how many 0x78s we've tolerated
    std::vector<uint8_t> resp;

    while (true) {
        st = tp_.recv(resp);
        if (st != isotp::Status::Ok) { r.tp_status = st; break; }
        if (resp.empty())            { r.tp_status = isotp::Status::BadFrame; break; }

        // --- Case A: Negative response ---
        // Wire format: [ 0x7F, <echoed SID>, <NRC> ]
        // We also require the echoed SID to match what we asked for -- this
        // filters out stray chatter from other diagnostic flows.
        if (resp.size() >= 3 && resp[0] == 0x7F && resp[1] == req[0]) {
            uint8_t nrc = resp[2];
            if (nrc == 0x78 && pending_count < max_response_pending) {
                // "Still processing" -- loop and read again.
                ++pending_count;
                if (verbose_) {
                    std::cout << "[UDS  RX] 0x78 response-pending ("
                              << pending_count << "/" << max_response_pending
                              << "), keep waiting..." << std::endl;
                }
                continue;
            }
            r.ok  = false;
            r.sid = 0x7F;
            r.nrc = nrc;
            break;
        }

        // --- Case B: Positive response ---
        // The ECU echoes our service ID + 0x40 as the first byte. Example:
        //   we sent 0x22 (ReadDataByIdentifier), we get back 0x62.
        //   we sent 0x2E (WriteDataByIdentifier), we get back 0x6E.
        //   we sent 0x10 (DiagnosticSessionControl), we get back 0x50.
        // Everything after that first byte is the service-specific data.
        if (resp[0] == static_cast<uint8_t>(req[0] + 0x40)) {
            r.ok  = true;
            r.sid = resp[0];
            r.data.assign(resp.begin() + 1, resp.end());
            break;
        }

        // --- Case C: Not our response at all ---
        // We got *something* that is neither a matching positive response
        // nor a matching negative response. Treat as garbage and bail.
        r.tp_status = isotp::Status::BadFrame;
        break;
    }

    // Restore the original transport timeout for the next call.
    cfg.timeout_ms = saved;
    tp_.setConfig(cfg);

    logResp(r);
    return r;
}

// ---------------------------------------------------------------------------
// Service-specific wrappers
// ---------------------------------------------------------------------------
Response UdsClient::diagnosticSessionControl(uint8_t session) {
    return request({ SID::DIAGNOSTIC_SESSION_CONTROL, session });
}

Response UdsClient::testerPresent(bool suppress_pos_response) {
    // 0x3E 0x00 by default; 0x3E 0x80 adds the "suppress positive response"
    // bit so we don't even get an answer -- useful for keep-alive threads.
    uint8_t sub = suppress_pos_response ? 0x80 : 0x00;
    if (suppress_pos_response) {
        // Fire and forget at transport layer.
        std::vector<uint8_t> req{ SID::TESTER_PRESENT, sub };
        logReq(req);
        isotp::Status st = tp_.send(req);
        Response r{};
        r.tp_status = st;
        r.ok = (st == isotp::Status::Ok);
        r.sid = SID::TESTER_PRESENT + 0x40;
        return r;
    }
    return request({ SID::TESTER_PRESENT, sub });
}

// RDBI (0x22): read one resource, addressed by a 16-bit DID.
// Wire format:  [ 0x22, DID_high, DID_low ]
// We split the 16-bit DID into two bytes the same way you'd do it in JS:
//   high = (did >> 8) & 0xFF;   // top 8 bits
//   low  =  did       & 0xFF;   // bottom 8 bits
Response UdsClient::readDataByIdentifier(uint16_t did) {
    return request({
        SID::READ_DATA_BY_IDENTIFIER,
        static_cast<uint8_t>((did >> 8) & 0xFF),
        static_cast<uint8_t>(did & 0xFF)
    }, 3000);
}

// WDBI (0x2E): write one resource, addressed by a 16-bit DID.
// Wire format:  [ 0x2E, DID_high, DID_low, ...data bytes... ]
//
// We allow longer timeouts here than for reads because writes often trigger
// an EEPROM commit inside the ECU, which can take hundreds of milliseconds
// and is usually preceded by a storm of NRC 0x78 ("still processing") replies.
Response UdsClient::writeDataByIdentifier(uint16_t did,
                                          const std::vector<uint8_t>& data) {
    std::vector<uint8_t> req;
    req.reserve(3 + data.size());
    req.push_back(SID::WRITE_DATA_BY_IDENTIFIER);
    req.push_back(static_cast<uint8_t>((did >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(did & 0xFF));
    req.insert(req.end(), data.begin(), data.end());
    return request(req, /*response_timeout_ms=*/5000, /*max_response_pending=*/20);
}

Response UdsClient::securityAccess(uint8_t req_seed_sub, KeyFn key_fn) {
    if ((req_seed_sub & 0x01) == 0) {
        Response r{};
        r.tp_status = isotp::Status::BadFrame;
        return r;
    }

    // Step 1: request seed.
    Response seed_r = request({ SID::SECURITY_ACCESS, req_seed_sub });
    if (!seed_r.ok) return seed_r;

    // First byte of the positive response payload is the echoed sub-function,
    // the rest is the seed.
    if (seed_r.data.empty()) {
        Response bad{};
        bad.tp_status = isotp::Status::BadFrame;
        return bad;
    }
    std::vector<uint8_t> seed(seed_r.data.begin() + 1, seed_r.data.end());

    // All-zero seed means "already unlocked at this level".
    bool all_zero = !seed.empty();
    for (auto b : seed) if (b != 0) { all_zero = false; break; }
    if (seed.empty() || all_zero) {
        if (verbose_) {
            std::cout << "[UDS] Security level already unlocked (seed all-zero)."
                      << std::endl;
        }
        return seed_r;
    }

    if (verbose_) {
        std::cout << "[UDS] Seed: " << hexu::bytesToHex(seed) << std::endl;
    }

    // Step 2: compute and send key.
    std::vector<uint8_t> key = key_fn(seed);
    if (verbose_) {
        std::cout << "[UDS] Key:  " << hexu::bytesToHex(key) << std::endl;
    }

    std::vector<uint8_t> req;
    req.reserve(2 + key.size());
    req.push_back(SID::SECURITY_ACCESS);
    req.push_back(static_cast<uint8_t>(req_seed_sub + 1));
    req.insert(req.end(), key.begin(), key.end());
    return request(req);
}

} // namespace uds
