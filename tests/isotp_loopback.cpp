// End-to-end loopback test for iso_tp + uds_client + vw_mqb + backup.
//
// Uses a SOCK_SEQPACKET AF_UNIX socketpair that preserves whole
// `struct can_frame`-sized messages. A programmable fake ECU thread handles
// UDS requests using the same IsoTp code with TX/RX IDs swapped. Between
// test sections the main thread reconfigures the ECU (NRC to inject,
// response-pending storm length, readback corruption, etc.) so every error
// path gets exercised without needing a real car.
//
// In addition to protocol round-trips, the file contains direct unit tests
// for pure-function helpers (nrcToString, statusToString, hex utilities,
// VW key helpers) so coverage isn't gated on being able to trigger a
// particular NRC over the bus.

#include "backup.h"
#include "hex_utils.h"
#include "iso_tp.h"
#include "uds_client.h"
#include "vw_mqb.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/can.h>

namespace {

// ============================================================================
// Programmable fake ECU state.
//
// Tests toggle these flags between phases to force specific error paths
// without needing a real car. Atomics keep the ECU thread and the test
// thread race-free.
// ============================================================================
struct EcuScript {
    // If >0, the ECU replies with NRC 0x78 ("ResponsePending") this many
    // times, then serves the real answer. Used to cover the pending loop.
    std::atomic<int>     pending_bursts{0};

    // If true, after emitting `pending_bursts` NRC 0x78 frames the ECU does
    // NOT follow up with a real answer. Used to cover the "max_response_pending
    // exhausted" branch without leaving stale replies in the socket buffer.
    std::atomic<bool>    pending_no_final_reply{false};

    // If non-zero, override matching responses with [0x7F, SID, nrc]. The
    // flag is cleared when force_nrc_count hits zero (default 1 = just the
    // next request). Use >1 to fail several RDBIs in a row (useful for
    // multi-read helpers like readGatewayIdentity).
    std::atomic<uint8_t> force_nrc{0};
    std::atomic<uint8_t> force_nrc_for_sid{0};   // 0 = any SID; else only this one
    std::atomic<int>     force_nrc_count{1};
    // Skip this many matching requests BEFORE starting to inject the NRC.
    // "Make the second 0x2E fail but the first succeed" -> skip=1, count=1.
    std::atomic<int>     force_nrc_skip{0};

    // If true, accept WDBI (ACK 0x6E) but do NOT update the stored state,
    // so a readback returns a different value. Exercises readback-mismatch.
    std::atomic<bool>    wdbi_silent_drop{false};
    // If >0, silently drop this many WRITE requests (2E) AFTER skipping the
    // preceding ones normally. Lets us target the second WDBI in a restore.
    std::atomic<int>     wdbi_silent_drop_skip{0};

    // If true, RDBI returns a truncated response (just "62 DIDhi"), too
    // short for the UDS client to interpret.
    std::atomic<bool>    rdbi_truncate{false};

    // If true, the NEXT RDBI response is ONLY [0x62] (no DID echo, no data).
    // Exercises `r.data.size() < 2` safety branches in vw_mqb and backup.
    std::atomic<bool>    rdbi_ultra_truncate{false};

    // If non-zero, the next RDBI matching this specific DID is returned as
    // [0x62, DIDhi] (r.data.size() == 1 at the UDS layer). Targets the
    // "Coding response too short" branch in backup::capture without
    // disrupting preceding identity reads.
    std::atomic<uint16_t> truncate_rdbi_did{0};

    // If true, next Installation List read returns < 0x76 bytes (exercises
    // the "Gateway layout unfamiliar, refusing to write" error path).
    std::atomic<bool>    short_install_list{false};

    // If true, SecurityAccess returns a NON-zero seed (so the client runs
    // the real keyFn branch rather than "already unlocked" fast path).
    std::atomic<bool>    sec_real_seed{false};

    // If true, next SecurityAccess positive response carries no data at all
    // (just [0x67]). Exercises the "seed_r.data.empty() -> BadFrame" branch.
    std::atomic<bool>    sec_empty_response{false};

    // If true, the positive response for the *next* request has a wrong
    // SID echo (to trigger the "not our response" BadFrame branch).
    std::atomic<bool>    wrong_sid_echo{false};
};

static std::atomic<bool> g_stop{false};

void ecuThread(int fd, EcuScript& sc) {
    isotp::Config cfg;
    cfg.tx_id       = 0x18DAF110;
    cfg.rx_id       = 0x18DA10F1;
    cfg.extended_id = true;
    cfg.padding     = 0xAA;
    cfg.timeout_ms  = 300;
    cfg.verbose     = false;
    cfg.rx_st_min   = 0;      // default: fire-hose CFs; raised in one test

    isotp::IsoTp tp(fd, cfg);

    std::vector<uint8_t> coding = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x2F, 0x10
    };
    std::vector<uint8_t> install_list(0x80, 0x00);
    install_list[0x01] = 0x01; install_list[0x19] = 0x01; install_list[0x75] = 0x01;

    while (!g_stop.load()) {
        std::vector<uint8_t> req;
        auto st = tp.recv(req);
        if (st == isotp::Status::TimeoutRx) continue;
        if (st != isotp::Status::Ok)        break;
        if (req.empty())                    continue;

        uint8_t sid = req[0];

        // Pending bursts: reply 0x78 N times (UDS client does NOT retransmit on
        // 0x78 -- it simply re-reads -- so we must emit all N "pending" NRCs
        // consecutively, then the real answer, all for the same request.
        {
            int bursts = sc.pending_bursts.exchange(0);
            bool transport_ok = true;
            for (int i = 0; i < bursts && transport_ok; ++i) {
                std::vector<uint8_t> neg = { 0x7F, sid, 0x78 };
                if (tp.send(neg) != isotp::Status::Ok) transport_ok = false;
            }
            if (!transport_ok) break;
            if (bursts > 0 && sc.pending_no_final_reply.exchange(false)) {
                // Exhaustion test: skip real answer so no stale bytes linger.
                continue;
            }
        }

        // Forced NRC injection. Supports "skip N requests, then fail next M".
        uint8_t fnrc = sc.force_nrc.load();
        uint8_t fsid = sc.force_nrc_for_sid.load();
        if (fnrc != 0 && (fsid == 0 || fsid == sid)) {
            int skip = sc.force_nrc_skip.load();
            if (skip > 0) {
                sc.force_nrc_skip.fetch_sub(1);
                // fall through: handle this request normally
            } else {
                int remaining = sc.force_nrc_count.fetch_sub(1);
                if (remaining <= 1) {
                    sc.force_nrc.store(0);
                    sc.force_nrc_for_sid.store(0);
                    sc.force_nrc_count.store(1);
                }
                std::vector<uint8_t> neg = { 0x7F, sid, fnrc };
                if (tp.send(neg) != isotp::Status::Ok) break;
                continue;
            }
        }

        std::vector<uint8_t> resp;
        switch (sid) {
            case 0x10: {
                uint8_t sub = (req.size() > 1) ? req[1] : uint8_t{0x01};
                resp = { 0x50, sub, 0x00, 0x32, 0x01, 0xF4 };
                break;
            }
            case 0x22: {
                if (req.size() < 3) { resp = { 0x7F, 0x22, 0x13 }; break; }
                uint16_t did = (uint16_t(req[1]) << 8) | req[2];
                if (sc.rdbi_ultra_truncate.exchange(false)) {
                    // Pathological: just the SID echo, no DID echo at all.
                    // Covers `r.data.size() < 2` guards.
                    resp = { 0x62 };
                    break;
                }
                resp = { 0x62, req[1], req[2] };
                if (sc.rdbi_truncate.exchange(false)) {
                    // Truncated response: only SID+DID echo, no payload data.
                    break;
                }
                if (sc.truncate_rdbi_did.load() == did) {
                    sc.truncate_rdbi_did.store(0);
                    // Return [0x62, DIDhi] -> at UDS layer r.data.size() == 1.
                    resp.pop_back();
                    break;
                }
                if      (did == 0xF190) for (char c : std::string("WVGBV7AX7LK000001")) resp.push_back(uint8_t(c));
                else if (did == 0xF187) for (char c : std::string("5Q0907530D"))        resp.push_back(uint8_t(c));
                else if (did == 0xF189) for (char c : std::string("0271"))              resp.push_back(uint8_t(c));
                else if (did == 0xF18C) for (char c : std::string("SN12345678"))        resp.push_back(uint8_t(c));
                else if (did == 0x0608) {
                    if (sc.short_install_list.exchange(false)) {
                        std::vector<uint8_t> shorter(0x50, 0x00);
                        resp.insert(resp.end(), shorter.begin(), shorter.end());
                    } else {
                        resp.insert(resp.end(), install_list.begin(), install_list.end());
                    }
                }
                else if (did == 0x0600) resp.insert(resp.end(), coding.begin(), coding.end());
                else                    resp = { 0x7F, 0x22, 0x31 };
                break;
            }
            case 0x2E: {
                if (req.size() < 3) { resp = { 0x7F, 0x2E, 0x13 }; break; }
                uint16_t did = (uint16_t(req[1]) << 8) | req[2];
                std::vector<uint8_t> payload(req.begin() + 3, req.end());
                bool drop = sc.wdbi_silent_drop.exchange(false);
                int drop_skip = sc.wdbi_silent_drop_skip.load();
                if (drop_skip > 0) {
                    if (drop_skip == 1) drop = true;   // THIS write is the target
                    sc.wdbi_silent_drop_skip.fetch_sub(1);
                }
                if (did == 0x0600) {
                    if (payload.size() != coding.size())        { resp = { 0x7F, 0x2E, 0x13 }; break; }
                    if (!drop) coding = payload;
                    resp = { 0x6E, req[1], req[2] };
                } else if (did == 0x0608) {
                    if (payload.size() != install_list.size())  { resp = { 0x7F, 0x2E, 0x13 }; break; }
                    if (!drop) install_list = payload;
                    resp = { 0x6E, req[1], req[2] };
                } else {
                    resp = { 0x6E, req[1], req[2] };
                }
                break;
            }
            case 0x27: {
                if (req.size() < 2) { resp = { 0x7F, 0x27, 0x13 }; break; }
                if (sc.sec_empty_response.exchange(false)) {
                    // Pathological: positive response with no payload at all.
                    resp = { 0x67 };
                } else if (req[1] & 0x01) {
                    if (sc.sec_real_seed.load()) {
                        resp = { 0x67, req[1], 0xDE, 0xAD, 0xBE, 0xEF };
                    } else {
                        resp = { 0x67, req[1], 0x00, 0x00, 0x00, 0x00 };
                    }
                } else {
                    resp = { 0x67, req[1] };
                }
                break;
            }
            case 0x3E: {
                if ((req.size() > 1) && (req[1] & 0x80)) continue; // no reply
                uint8_t sub = (req.size() > 1) ? uint8_t(req[1] & 0x7F) : uint8_t{0x00};
                resp = { 0x7E, sub };
                break;
            }
            default:
                resp = { 0x7F, sid, 0x11 };
                break;
        }

        if (sc.wrong_sid_echo.exchange(false) && !resp.empty()) {
            resp[0] = 0xFF; // garbage SID echo
        }

        if (tp.send(resp) != isotp::Status::Ok) break;
    }
}

// ============================================================================
// Verdict counter
// ============================================================================
struct Verdict {
    int failures = 0;
    void operator()(const std::string& name, bool ok, const std::string& detail = "") {
        std::cout << (ok ? "PASS: " : "FAIL: ") << name;
        if (!detail.empty()) std::cout << "   (" << detail << ")";
        std::cout << "\n";
        if (!ok) ++failures;
    }
};

// ============================================================================
// Section A: direct unit tests for pure helpers (no socket needed).
// These drive every NRC case, every Status case, the hex utilities, and the
// VW key helpers -- closing a lot of gcov gaps in one shot.
// ============================================================================
void sectionPureHelpers(Verdict& v) {
    // --- hex utils ---
    {
        std::vector<uint8_t> in{0x01, 0x02, 0x0A, 0xFF};
        v("bytesToHex(vector) format",         hexu::bytesToHex(in) == "01 02 0A FF");
        v("bytesToHex(ptr, n)",                 hexu::bytesToHex(in.data(), in.size()) == "01 02 0A FF");
        v("bytesToHex empty",                   hexu::bytesToHex(std::vector<uint8_t>{}).empty());
        v("idToHex 11-bit",                     hexu::idToHex(0x7E0, false) == "7E0");
        v("idToHex 29-bit",                     hexu::idToHex(0x18DAF110, true) == "18DAF110");

        std::vector<uint8_t> out;
        v("parseHexString valid",               hexu::parseHexString("01 02 0A FF", out) && out == in);
        v("parseHexString no-space",            hexu::parseHexString("01020AFF", out) && out.size() == 4);
        std::vector<uint8_t> junk;
        v("parseHexString odd -> false",        !hexu::parseHexString("0 1 2", junk));
        v("parseHexString non-hex -> false",    !hexu::parseHexString("ZZ", junk));
    }

    // --- NRC decoding ---  We construct a response by hand and call describe().
    auto nrcMessage = [](uint8_t nrc) {
        uds::Response r{};
        r.ok  = false;
        r.sid = 0x7F;
        r.nrc = nrc;
        return r.describe();
    };
    v("NRC 0x10 generalReject",                 nrcMessage(0x10).find("generalReject") != std::string::npos);
    v("NRC 0x11 serviceNotSupported",           nrcMessage(0x11).find("serviceNotSupported") != std::string::npos);
    v("NRC 0x12 subFunctionNotSupported",       nrcMessage(0x12).find("subFunctionNotSupported") != std::string::npos);
    v("NRC 0x13 incorrectMessageLength",        nrcMessage(0x13).find("incorrectMessageLength") != std::string::npos);
    v("NRC 0x14 responseTooLong",               nrcMessage(0x14).find("responseTooLong") != std::string::npos);
    v("NRC 0x21 busyRepeatRequest",             nrcMessage(0x21).find("busyRepeatRequest") != std::string::npos);
    v("NRC 0x22 conditionsNotCorrect",          nrcMessage(0x22).find("conditionsNotCorrect") != std::string::npos);
    v("NRC 0x24 requestSequenceError",          nrcMessage(0x24).find("requestSequenceError") != std::string::npos);
    v("NRC 0x25 noResponseFromSubnetComponent", nrcMessage(0x25).find("noResponseFromSubnetComponent") != std::string::npos);
    v("NRC 0x26 failurePreventsExecution",      nrcMessage(0x26).find("failurePreventsExecution") != std::string::npos);
    v("NRC 0x31 requestOutOfRange",             nrcMessage(0x31).find("requestOutOfRange") != std::string::npos);
    v("NRC 0x33 securityAccessDenied",          nrcMessage(0x33).find("securityAccessDenied") != std::string::npos);
    v("NRC 0x35 invalidKey",                    nrcMessage(0x35).find("invalidKey") != std::string::npos);
    v("NRC 0x36 exceededNumberOfAttempts",      nrcMessage(0x36).find("exceededNumberOfAttempts") != std::string::npos);
    v("NRC 0x37 requiredTimeDelayNotExpired",   nrcMessage(0x37).find("requiredTimeDelayNotExpired") != std::string::npos);
    v("NRC 0x70 uploadDownloadNotAccepted",     nrcMessage(0x70).find("uploadDownloadNotAccepted") != std::string::npos);
    v("NRC 0x71 transferDataSuspended",         nrcMessage(0x71).find("transferDataSuspended") != std::string::npos);
    v("NRC 0x72 generalProgrammingFailure",     nrcMessage(0x72).find("generalProgrammingFailure") != std::string::npos);
    v("NRC 0x73 wrongBlockSequenceCounter",     nrcMessage(0x73).find("wrongBlockSequenceCounter") != std::string::npos);
    v("NRC 0x78 ResponsePending",               nrcMessage(0x78).find("ResponsePending") != std::string::npos);
    v("NRC 0x7E subFunctionNotSupportedInActive", nrcMessage(0x7E).find("subFunctionNotSupportedInActiveSession") != std::string::npos);
    v("NRC 0x7F serviceNotSupportedInActive",   nrcMessage(0x7F).find("serviceNotSupportedInActiveSession") != std::string::npos);
    v("NRC 0xAA unknown",                       nrcMessage(0xAA).find("unknown") != std::string::npos);

    // Positive response describe()
    {
        uds::Response r{};
        r.ok   = true;
        r.sid  = 0x62;
        r.data = {0xF1, 0x90, 'W','V','G'};
        v("Response::describe positive",       r.describe().find("OK") != std::string::npos);
    }

    // Transport-error describe()
    {
        uds::Response r{};
        r.ok        = false;
        r.sid       = 0;
        r.nrc       = 0;
        r.tp_status = isotp::Status::TimeoutRx;
        v("Response::describe transport-error",
          r.describe().find("Transport error") != std::string::npos
              && r.describe().find("TimeoutRx") != std::string::npos);
    }

    // Every Status string value.
    using isotp::Status;
    using isotp::statusToString;
    v("statusToString Ok",            std::string(statusToString(Status::Ok))            == "Ok");
    v("statusToString TimeoutRx",     std::string(statusToString(Status::TimeoutRx))     == "TimeoutRx");
    v("statusToString TimeoutFc",     std::string(statusToString(Status::TimeoutFc))     == "TimeoutFc");
    v("statusToString SocketError",   std::string(statusToString(Status::SocketError))   == "SocketError");
    v("statusToString UnexpectedPci", std::string(statusToString(Status::UnexpectedPci)) == "UnexpectedPci");
    v("statusToString SequenceError", std::string(statusToString(Status::SequenceError)) == "SequenceError");
    v("statusToString OverflowFc",    std::string(statusToString(Status::OverflowFc))    == "OverflowFc");
    v("statusToString TooLong",       std::string(statusToString(Status::TooLong))       == "TooLong");
    v("statusToString BadFrame",      std::string(statusToString(Status::BadFrame))      == "BadFrame");
    v("statusToString Unknown",       std::string(statusToString(static_cast<Status>(99))) == "Unknown");

    // VW key helpers
    {
        auto k = vw::keyFromStaticLogin(0x11223344);
        v("keyFromStaticLogin big-endian", k.size() == 4 && k[0] == 0x11 && k[1] == 0x22 && k[2] == 0x33 && k[3] == 0x44);
        auto k2 = vw::keyFromSeedPlaceholder({0x00, 0xFF, 0xA5});
        v("keyFromSeedPlaceholder XOR",    k2.size() == 3 && k2[0] == 0xFF && k2[1] == 0x00 && k2[2] == 0x5A);
    }

    // InstallationList bit helpers (both set and clear directions).
    {
        vw::InstallationList l;
        l.raw.assign(4, 0x00);
        v("isInstalled out-of-range -> false", !l.isInstalled(99));
        v("setInstalled out-of-range -> false", !l.setInstalled(99, true));
        l.setInstalled(2, true);
        v("setInstalled(true) sets bit 0",      l.isInstalled(2));
        l.setInstalled(2, false);
        v("setInstalled(false) clears bit 0",   !l.isInstalled(2));
    }

    // backup::print writes to stdout; just call it for coverage.
    {
        backup::Data d;
        d.vin = "X"; d.part_number = "PN"; d.sw_version = "V"; d.serial = "S"; d.timestamp = "T";
        d.coding.assign(2, 0xAB);
        d.installation_list.assign(0x80, 0x00);
        backup::print(d);
        v("backup::print executed", true);
    }
}

// ============================================================================
// Section B: file-I/O and parsing tests for backup.cpp error branches.
// ============================================================================
void sectionBackupFiles(Verdict& v) {
    // Failure: load non-existent file.
    backup::Data d;
    v("backup::load missing file -> false", !backup::load("/tmp/elm-nope-XYZ123.txt", d));

    // Failure: file missing required fields.
    {
        const char* p = "/tmp/elm-empty.txt";
        FILE* f = std::fopen(p, "w"); std::fputs("# empty\n", f); std::fclose(f);
        v("backup::load empty file -> false", !backup::load(p, d));
        std::remove(p);
    }

    // Failure: malformed coding hex.
    {
        const char* p = "/tmp/elm-badhex.txt";
        FILE* f = std::fopen(p, "w");
        std::fputs("vin=X\npart_number=Y\ncoding=ZZZZ\ninstallation_list=00\n", f);
        std::fclose(f);
        v("backup::load bad coding hex -> false", !backup::load(p, d));
        std::remove(p);
    }

    // Failure: malformed installation_list hex.
    {
        const char* p = "/tmp/elm-badlist.txt";
        FILE* f = std::fopen(p, "w");
        std::fputs("vin=X\npart_number=Y\ncoding=01\ninstallation_list=ZZ\n", f);
        std::fclose(f);
        v("backup::load bad list hex -> false", !backup::load(p, d));
        std::remove(p);
    }

    // Failure: save to unwritable path.
    {
        backup::Data ok;
        ok.vin = "X"; ok.part_number = "Y";
        ok.coding.assign(2, 1);
        ok.installation_list.assign(2, 2);
        v("backup::save bad path -> false",
          !backup::save("/nonexistent/dir/xyz.txt", ok));
    }
}

// ============================================================================
// Section C: integration tests that talk to the programmable fake ECU.
// Each block reconfigures the script between phases.
// ============================================================================
void sectionIntegration(int fd, EcuScript& sc, Verdict& v) {
    isotp::Config cfg;
    cfg.tx_id       = 0x18DA10F1;
    cfg.rx_id       = 0x18DAF110;
    cfg.extended_id = true;
    cfg.padding     = 0xAA;
    cfg.timeout_ms  = 2000;
    cfg.verbose     = true;

    uds::UdsClient client(fd, cfg);

    // ---- 1. Baseline SF/SF ----
    v("Baseline SF/SF DiagSession(Ext)", client.diagnosticSessionControl(0x03).ok);

    // ---- 2. SF request -> FF+CF response (VIN) ----
    v("RDBI F190 VIN (multi-frame RX)", client.readDataByIdentifier(0xF190).ok);

    // ---- 3. 0x78 response-pending loop ----
    {
        sc.pending_bursts.store(3);    // ECU sends 0x78 three times before real answer
        auto r = client.diagnosticSessionControl(0x03);
        v("0x78 pending loop accepted (3 bursts)", r.ok);
    }

    // ---- 4. 0x78 loop exceeds max -> failure ----
    // Default max_response_pending for request() is 10. Send 11 bursts and
    // suppress the real reply so the client sees exactly 11 NRC 0x78 and
    // aborts at the 11th without any stale bytes piling up on the bus.
    {
        sc.pending_no_final_reply.store(true);
        sc.pending_bursts.store(11);
        auto r = client.diagnosticSessionControl(0x03);
        v("0x78 loop exhausted -> NACK", !r.ok && r.nrc == 0x78);
    }

    // ---- 5. Force NRC on next RDBI ----
    {
        sc.force_nrc.store(0x33);
        sc.force_nrc_for_sid.store(0x22);
        auto r = client.readDataByIdentifier(0x0600);
        v("Forced NRC 0x33 propagates", !r.ok && r.nrc == 0x33);
    }

    // ---- 6. Wrong SID echo -> BadFrame ----
    {
        sc.wrong_sid_echo.store(true);
        auto r = client.readDataByIdentifier(0xF190);
        v("Wrong SID echo -> BadFrame", !r.ok && r.tp_status == isotp::Status::BadFrame);
    }

    // ---- 7. TesterPresent with suppress bit (fire-and-forget path) ----
    {
        auto r = client.testerPresent(true);
        v("TesterPresent suppress-pos-response", r.ok);
    }

    // ---- 9. SecurityAccess with non-zero seed -> runs keyFn branch ----
    {
        sc.sec_real_seed.store(true);
        auto r = client.securityAccess(0x03, [](const std::vector<uint8_t>& seed) {
            std::vector<uint8_t> k = seed;
            for (auto& b : k) b ^= 0xFF;
            return k;
        });
        sc.sec_real_seed.store(false);
        v("SecurityAccess real-seed key send", r.ok);
    }

    // ---- 10. SecurityAccess with even sub-function -> BadFrame ----
    {
        auto r = client.securityAccess(0x02, [](auto& s){ return s; });
        v("SecurityAccess even sub -> BadFrame",
          !r.ok && r.tp_status == isotp::Status::BadFrame);
    }

    // ---- 11. vw_mqb helpers (reads only first, list/ident/dry-run) ----
    {
        vw::GatewayIdentity id;
        v("vw::readGatewayIdentity",
          vw::readGatewayIdentity(client, id)
              && id.vin == "WVGBV7AX7LK000001"
              && id.part_number == "5Q0907530D");

        vw::InstallationList list;
        v("vw::readInstallationList",
          vw::readInstallationList(client, list)
              && list.raw.size() == 0x80 && list.isInstalled(0x75));

        std::ostringstream sink;
        vw::printInstallationList(list, sink);
        v("vw::printInstallationList produced output", sink.str().size() > 100);

        v("vw::fixEmergencyCall(dry_run)",      vw::fixEmergencyCall(client, true));
        v("vw::disableStartStop(dry_run)",      vw::disableStartStop(client, true, 14, 0x20, true));
    }

    // ---- 12. vw::readInstallationList RDBI-error branch ----
    {
        sc.force_nrc.store(0x33);
        sc.force_nrc_for_sid.store(0x22);
        vw::InstallationList list;
        v("readInstallationList returns false on NRC", !vw::readInstallationList(client, list));
    }

    // ---- 13. vw::readInstallationList too-short branch ----
    // rdbi_ultra_truncate returns just [0x62] so r.data is empty (< 2).
    {
        sc.rdbi_ultra_truncate.store(true);
        vw::InstallationList list;
        v("readInstallationList returns false on truncated response",
          !vw::readInstallationList(client, list));
    }

    // ---- 14. Backup round-trip end-to-end ----
    const char* BACKUP_PATH = "/tmp/elm-uds-backup.txt";
    backup::Data before;
    {
        v("backup::capture", backup::capture(client, before) && before.vin == "WVGBV7AX7LK000001");
        v("backup::save",    backup::save(BACKUP_PATH, before));
        backup::Data loaded;
        v("backup::load round-trip", backup::load(BACKUP_PATH, loaded)
                                     && loaded.coding == before.coding
                                     && loaded.installation_list == before.installation_list);
        bool sw_changed = false;
        v("verifyIdentity matches", backup::verifyIdentity(client, loaded, sw_changed));
    }

    // ---- 15. verifyIdentity VIN-mismatch ----
    {
        backup::Data bad = before;
        bad.vin = "ZZZZZZZZZZZZZZZZZ";
        bool sw_changed = false;
        v("verifyIdentity VIN mismatch -> false", !backup::verifyIdentity(client, bad, sw_changed));
    }

    // ---- 16. verifyIdentity part-number mismatch ----
    {
        backup::Data bad = before;
        bad.part_number = "OTHER_PN";
        bool sw_changed = false;
        v("verifyIdentity part-number mismatch -> false", !backup::verifyIdentity(client, bad, sw_changed));
    }

    // ---- 17. verifyIdentity SW version change (warning, not failure) ----
    {
        backup::Data warn = before;
        warn.sw_version = "9999";
        bool sw_changed = false;
        v("verifyIdentity SW change -> true + sw_changed flag",
          backup::verifyIdentity(client, warn, sw_changed) && sw_changed);
    }

    // ---- 18. backup::capture with identity RDBI failing ----
    // readGatewayIdentity tries 4 RDBIs and only fails if VIN AND part-number
    // both failed. Force all 4 identity reads to NRC 0x33.
    {
        sc.force_nrc.store(0x33);
        sc.force_nrc_for_sid.store(0x22);
        sc.force_nrc_count.store(4);
        backup::Data d;
        v("backup::capture identity-fail -> false", !backup::capture(client, d));
    }

    // ---- 19. backup::restore dry-run path ----
    v("backup::restore dry_run", backup::restore(client, before, true));

    // ---- 20. fixEmergencyCall apply + readback OK ----
    {
        v("fixEmergencyCall apply OK + readback", vw::fixEmergencyCall(client, false));
        vw::InstallationList after;
        v("after apply: 0x75 gone",
          vw::readInstallationList(client, after) && !after.isInstalled(0x75));

        // Restore via backup::restore (exercises full apply-and-verify path).
        v("backup::restore apply round-trip", backup::restore(client, before, false));
        vw::InstallationList restored;
        v("after restore: 0x75 back",
          vw::readInstallationList(client, restored) && restored.isInstalled(0x75));
    }

    // ---- 21. fixEmergencyCall no-op (module 0x75 already cleared) ----
    {
        // Clear 0x75 first, verify already-not-installed branch returns true.
        vw::InstallationList cleared;
        v("read list pre-clear", vw::readInstallationList(client, cleared));
        cleared.setInstalled(0x75, false);
        v("write list via helper", vw::writeInstallationList(client, cleared, false));
        v("fixEmergencyCall no-op when already-cleared",
          vw::fixEmergencyCall(client, false));
        // Put it back.
        cleared.setInstalled(0x75, true);
        vw::writeInstallationList(client, cleared, false);
    }

    // ---- 22. writeInstallationList WDBI-NRC branch ----
    {
        sc.force_nrc.store(0x72);
        sc.force_nrc_for_sid.store(0x2E);
        vw::InstallationList l;
        vw::readInstallationList(client, l);
        v("writeInstallationList NRC -> false", !vw::writeInstallationList(client, l, false));
    }

    // ---- 23. writeInstallationList readback-mismatch branch ----
    {
        sc.wdbi_silent_drop.store(true);
        vw::InstallationList l;
        vw::readInstallationList(client, l);
        l.setInstalled(0x75, false);
        v("writeInstallationList readback-mismatch -> false",
          !vw::writeInstallationList(client, l, false));
    }

    // ---- 24. disableStartStop apply OK path (with readback-verify) ----
    {
        v("disableStartStop apply OK",
          vw::disableStartStop(client, false, 14, 0x02, true));
    }

    // ---- 25. disableStartStop no-op (already-in-requested-state) ----
    {
        // Bit 0x02 is already cleared from test 24, so setting clear_bit again is a no-op.
        v("disableStartStop no-op when already-set",
          vw::disableStartStop(client, false, 14, 0x02, true));
    }

    // ---- 26. disableStartStop RDBI NRC -> false ----
    {
        sc.force_nrc.store(0x22);
        sc.force_nrc_for_sid.store(0x22);
        v("disableStartStop RDBI NRC -> false",
          !vw::disableStartStop(client, false, 14, 0x20, true));
    }

    // ---- 27. disableStartStop index out of range -> false ----
    v("disableStartStop OOB index -> false",
      !vw::disableStartStop(client, false, 999, 0x20, true));

    // ---- 28. disableStartStop WDBI NRC -> false ----
    {
        sc.force_nrc.store(0x13);
        sc.force_nrc_for_sid.store(0x2E);
        v("disableStartStop WDBI NRC -> false",
          !vw::disableStartStop(client, false, 0, 0x40, false));
    }

    // ---- 29. disableStartStop readback-mismatch -> false ----
    {
        sc.wdbi_silent_drop.store(true);
        v("disableStartStop readback-mismatch -> false",
          !vw::disableStartStop(client, false, 0, 0x40, false));
    }

    // ---- 30. restore with coding-write NRC -> false ----
    {
        sc.force_nrc.store(0x72);
        sc.force_nrc_for_sid.store(0x2E);   // hits the FIRST write (coding)
        v("backup::restore coding-write NRC -> false",
          !backup::restore(client, before, false));
    }

    // ---- 31. restore with coding-readback mismatch -> false ----
    {
        sc.wdbi_silent_drop.store(true);   // next 0x2E is silently dropped -> readback differs
        v("backup::restore coding readback mismatch -> false",
          !backup::restore(client, before, false));
    }

    // ---- 32. SecurityAccess non-suppress TesterPresent + simple PASS ----
    // Covers the `request({SID::TESTER_PRESENT, sub})` line in the non-suppress
    // branch of UdsClient::testerPresent.
    v("TesterPresent non-suppress positive response",
      client.testerPresent(false).ok);

    // ---- 33. SecurityAccess returning no data (empty payload) -> BadFrame ----
    {
        sc.sec_empty_response.store(true);
        auto r = client.securityAccess(0x01, [](auto& s){ return s; });
        v("SecurityAccess empty seed payload -> BadFrame",
          !r.ok && r.tp_status == isotp::Status::BadFrame);
    }

    // ---- 34. SecurityAccess already unlocked (all-zero seed) ----
    {
        auto r = client.securityAccess(0x01, [](auto& s){ return s; });
        v("SecurityAccess all-zero seed -> already-unlocked path", r.ok);
    }

    // ---- 35. backup::capture: identity OK, coding RDBI fails ----
    // readGatewayIdentity issues 4 RDBIs; skip those and fail the 5th.
    {
        sc.force_nrc.store(0x22);
        sc.force_nrc_for_sid.store(0x22);
        sc.force_nrc_skip.store(4);
        sc.force_nrc_count.store(1);
        backup::Data d;
        v("backup::capture coding-RDBI-fail -> false", !backup::capture(client, d));
    }

    // ---- 36. backup::capture: coding response too short (1-byte payload) ----
    // truncate_rdbi_did affects only the next RDBI with DID 0x0600, leaving
    // the identity reads intact.
    {
        sc.truncate_rdbi_did.store(0x0600);
        backup::Data d;
        v("backup::capture coding-too-short -> false", !backup::capture(client, d));
    }

    // ---- 37. vw::readGatewayIdentity with ULTRA-truncated first response ----
    // Forces r.data.size() <= 2 for VIN so the `.size() > 2` guard is hit.
    {
        sc.rdbi_ultra_truncate.store(true);
        vw::GatewayIdentity id;
        // First RDBI gets [0x62] only (r.data empty), second onward succeed.
        (void)vw::readGatewayIdentity(client, id);
        v("readGatewayIdentity partial-data", id.part_number == "5Q0907530D" && id.vin.empty());
    }

    // ---- 38. writeInstallationList readback-RDBI failure ----
    // WDBI succeeds; the immediate readback RDBI fails with NRC. Covers the
    // "[VW] Readback failed:" branch in vw::writeInstallationList.
    {
        sc.force_nrc.store(0x22);
        sc.force_nrc_for_sid.store(0x22);
        sc.force_nrc_skip.store(0);
        vw::InstallationList l;
        // Put a clean list together for the write.
        l.raw.assign(0x80, 0x00);
        l.raw[0x75] = 0x01;
        // WDBI will succeed (first request that enters ECU with sid=0x2E),
        // then readback (sid=0x22) matches our forced NRC.
        v("writeInstallationList readback-RDBI-NRC -> false",
          !vw::writeInstallationList(client, l, false));
    }

    // ---- 39. fixEmergencyCall refuses short Installation List ----
    {
        sc.short_install_list.store(true);
        v("fixEmergencyCall refuses < 0x76 bytes list",
          !vw::fixEmergencyCall(client, false));
    }

    // ---- 40. disableStartStop ultra-truncated coding response ----
    // Coding RDBI returns [0x62] only (r.data.size() < 2). Hits the
    // "[VW] Coding response too short." branch.
    {
        sc.rdbi_ultra_truncate.store(true);
        v("disableStartStop coding-too-short -> false",
          !vw::disableStartStop(client, false, 0, 0x40, false));
    }

    // ---- 41. disableStartStop readback-RDBI failure ----
    // RDBI 0x0600 succeeds, WDBI 0x0600 succeeds, then readback RDBI fails.
    {
        sc.force_nrc.store(0x22);
        sc.force_nrc_for_sid.store(0x22);
        sc.force_nrc_skip.store(1);    // skip the first RDBI (initial read)
        sc.force_nrc_count.store(1);   // then fail the NEXT RDBI (readback)
        v("disableStartStop readback-RDBI NRC -> false",
          !vw::disableStartStop(client, false, 0, 0x40, false));
    }

    // ---- 42. backup::restore installation-list-write NRC ----
    // First 0x2E (coding) succeeds; second 0x2E (install list) gets NRC.
    {
        sc.force_nrc.store(0x72);
        sc.force_nrc_for_sid.store(0x2E);
        sc.force_nrc_skip.store(1);   // skip first WDBI, fail second
        sc.force_nrc_count.store(1);
        v("backup::restore install-list-write NRC -> false",
          !backup::restore(client, before, false));
    }

    // ---- 43. backup::restore installation-list readback mismatch ----
    // First WDBI (coding) commits normally; second WDBI (install list) is
    // silently dropped so readback returns the old data.
    {
        sc.wdbi_silent_drop_skip.store(2);  // skip 1, drop the 2nd
        v("backup::restore install-list readback-mismatch -> false",
          !backup::restore(client, before, false));
        // restore the known-good state for any trailing assertions
        backup::restore(client, before, false);
    }

    // ---- 44. backup::verifyIdentity when identity read itself fails ----
    {
        sc.force_nrc.store(0x22);
        sc.force_nrc_for_sid.store(0x22);
        sc.force_nrc_count.store(4);   // all 4 identity RDBIs fail
        backup::Data d = before;
        bool sw_changed = false;
        v("verifyIdentity identity-read-fail -> false",
          !backup::verifyIdentity(client, d, sw_changed));
    }

    // ---- 45. Final cleanup ----
    v("SF/SF DiagSession(Default)",
      client.diagnosticSessionControl(0x01).ok);
}

// ============================================================================
// Section D: dedicated scenario with rx_st_min > 0 so the sender hits the
// "sleep between CFs" branch in iso_tp.cpp.
//
// We run this twice with different STmin values:
//   * 0xF1 -> microsecond branch of decodeStMin (line 47)
//   * 0xFF -> reserved-value fallback of decodeStMin (line 48)
// ============================================================================
static void runStMinScenario(Verdict& v, uint8_t stmin, const char* label) {
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) < 0) {
        v(std::string("socketpair (stmin ") + label + ")", false);
        return;
    }

    std::atomic<bool> local_stop{false};
    std::thread ecu([&, stmin]{
        isotp::Config cfg;
        cfg.tx_id       = 0x18DAF110;
        cfg.rx_id       = 0x18DA10F1;
        cfg.extended_id = true;
        cfg.padding     = 0xAA;
        // Generous per-frame read timeout so CF gaps of up to 127ms (the
        // 0xFF fallback) don't trigger the ECU-side recv() timeout.
        cfg.timeout_ms  = 3000;
        cfg.verbose     = false;
        cfg.rx_st_min   = stmin;

        isotp::IsoTp tp(sv[1], cfg);

        while (!local_stop.load()) {
            std::vector<uint8_t> req;
            auto st = tp.recv(req);
            if (st == isotp::Status::TimeoutRx) continue;
            if (st != isotp::Status::Ok) break;
            if (req.empty()) continue;
            if (req[0] == 0x2E && req.size() >= 3) {
                std::vector<uint8_t> resp{ 0x6E, req[1], req[2] };
                tp.send(resp);
            } else {
                std::vector<uint8_t> resp{ 0x7F, req[0], 0x11 };
                tp.send(resp);
            }
        }
    });

    isotp::Config ccfg;
    ccfg.tx_id       = 0x18DA10F1;
    ccfg.rx_id       = 0x18DAF110;
    ccfg.extended_id = true;
    ccfg.padding     = 0xAA;
    ccfg.timeout_ms  = 8000;   // long enough for the worst-case 17*127ms send
    ccfg.verbose     = false;

    uds::UdsClient client(sv[0], ccfg);
    std::vector<uint8_t> big(0x80, 0xCD);
    auto r = client.writeDataByIdentifier(0x0608, big);
    v(std::string("STmin=") + label + " multi-frame TX + positive ACK", r.ok);

    local_stop.store(true);
    ::shutdown(sv[0], SHUT_RDWR);
    ::shutdown(sv[1], SHUT_RDWR);
    if (ecu.joinable()) ecu.join();
    ::close(sv[0]);
    ::close(sv[1]);
}

void sectionStMinPositive(Verdict& v) {
    runStMinScenario(v, 0xF1, "0xF1 (100us branch)");
    runStMinScenario(v, 0xFF, "0xFF (reserved fallback 127ms)");
}

// ============================================================================
// Section E: transport error path (socket closed mid-send/recv -> SocketError).
// ============================================================================
void sectionTransportError(Verdict& v) {
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) < 0) {
        v("socketpair (transport-err)", false);
        return;
    }

    isotp::Config cfg;
    cfg.tx_id       = 0x18DA10F1;
    cfg.rx_id       = 0x18DAF110;
    cfg.extended_id = true;
    cfg.padding     = 0xAA;
    cfg.timeout_ms  = 500;
    cfg.verbose     = false;

    // Close the peer so the next write gets EPIPE / ECONNRESET.
    ::close(sv[1]);

    uds::UdsClient client(sv[0], cfg);
    auto r = client.diagnosticSessionControl(0x03);
    v("send on closed peer -> transport error",
      !r.ok && r.tp_status != isotp::Status::Ok);

    ::close(sv[0]);
}

// ============================================================================
// Section F: `select() failed` path. We close the tester's own fd so the
// next select() returns EBADF, exercising the error branch in readCanFrame.
// ============================================================================
void sectionSelectFail(Verdict& v) {
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) < 0) {
        v("socketpair (select-fail)", false);
        return;
    }

    isotp::Config cfg;
    cfg.tx_id       = 0x18DA10F1;
    cfg.rx_id       = 0x18DAF110;
    cfg.extended_id = true;
    cfg.padding     = 0xAA;
    cfg.timeout_ms  = 200;
    cfg.verbose     = false;

    isotp::IsoTp tp(sv[0], cfg);
    // Close our own fd. The next select() call inside readCanFrame will
    // operate on an invalid descriptor and return EBADF.
    ::close(sv[0]);

    std::vector<uint8_t> out;
    auto st = tp.recv(out);
    v("recv on closed own-fd -> SocketError", st == isotp::Status::SocketError);

    ::close(sv[1]);
}

// ============================================================================
// Section G: inject a "Consecutive Frame as first frame" (0x2X) directly into
// the tester's socket. recv() should report Status::UnexpectedPci.
// ============================================================================
void sectionUnexpectedPci(Verdict& v) {
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) < 0) {
        v("socketpair (bad-pci)", false);
        return;
    }

    isotp::Config cfg;
    cfg.tx_id       = 0x18DA10F1;
    cfg.rx_id       = 0x18DAF110;
    cfg.extended_id = true;
    cfg.padding     = 0xAA;
    cfg.timeout_ms  = 1000;
    cfg.verbose     = false;

    isotp::IsoTp tp(sv[0], cfg);

    // Push a raw "Consecutive Frame" (PCI high nibble 0x2) onto the bus
    // WITHOUT a preceding First Frame. recv() must refuse with UnexpectedPci.
    struct can_frame f{};
    f.can_id  = 0x18DAF110 | 0x80000000u;  // EFF flag
    f.can_dlc = 8;
    uint8_t bad[8] = { 0x21, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00 };
    std::memcpy(f.data, bad, 8);
    if (::write(sv[1], &f, sizeof(f)) != static_cast<ssize_t>(sizeof(f))) {
        v("inject CF on ECU-side fd", false);
    } else {
        std::vector<uint8_t> out;
        auto st = tp.recv(out);
        v("CF as first frame -> UnexpectedPci", st == isotp::Status::UnexpectedPci);
    }

    ::close(sv[0]);
    ::close(sv[1]);
}

} // namespace

int main() {
    Verdict v;

    // Pure functions first.
    sectionPureHelpers(v);

    // File-I/O error branches.
    sectionBackupFiles(v);

    // Main integration (shares one ECU thread + socketpair).
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) < 0) {
        std::perror("socketpair"); return 2;
    }

    EcuScript sc;
    std::thread ecu(ecuThread, sv[1], std::ref(sc));

    sectionIntegration(sv[0], sc, v);

    g_stop.store(true);
    ::shutdown(sv[0], SHUT_RDWR);
    ::shutdown(sv[1], SHUT_RDWR);
    if (ecu.joinable()) ecu.join();
    ::close(sv[0]);
    ::close(sv[1]);

    // STmin > 0 scenario (separate socketpair, fresh ECU).
    sectionStMinPositive(v);

    // Socket error scenario.
    sectionTransportError(v);

    // select() failure scenario.
    sectionSelectFail(v);

    // Bad PCI (CF as first frame) scenario.
    sectionUnexpectedPci(v);

    std::cout << "\n" << (v.failures ? "OVERALL: FAIL" : "OVERALL: PASS")
              << " (" << v.failures << " failures)\n";
    return v.failures ? 1 : 0;
}
