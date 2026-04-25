// uds_main -- CLI front-end for the UDS client.
//
// High-level flow of every command:
//
//   1. Open a raw SocketCAN socket on `can0` and bind it.
//   2. Install a kernel filter so we only see frames from the ECU (rx_id).
//   3. Build an UdsClient on top of that fd. The UdsClient internally uses
//      an IsoTp instance -- the two speak ISO 15765-2 (chunking/reassembly).
//   4. Send the appropriate UDS service(s) and print results.
//
// See GUIDE.md for a plain-English walkthrough of what the bytes mean.
//
// Usage:
//   uds_main [--iface can0] [--tx 18DA10F1] [--rx 18DAF110] [--apply]
//            [--static-login 20103] [--sec-sub 03]
//            <command> [args...]
//
// Commands:
//   ident                          Print Gateway identification (VIN, PN, SW)
//   read-did <HEX>                 RDBI: read data by identifier (e.g. F190)
//   write-did <HEX> <BYTES>        WDBI: write data by identifier
//   dump-list                      Read DID 0x0608 and pretty-print it
//   dump-coding                    Read DID 0x0600 and pretty-print it
//   fix-ecall                      Remove Module 0x75 from Installation List
//   disable-ss                     Disable Start-Stop via Gateway coding
//   self-test                      Enter extended session, ping, leave
//   backup <FILE>                  Snapshot identity + coding + install list
//   restore <FILE>                 Verify identity, then write buffers back
//
// Safety policy:
//   * Without --apply, all write commands run in dry-run mode.
//   * With --apply, a fresh backup file is required and is verified against
//     the live ECU before any write is sent. Pass --backup <FILE>. The tool
//     refuses --apply without it (override with --no-backup at your own
//     risk; that flag exists for development / loopback testing only).

#include "backup.h"
#include "hex_utils.h"
#include "iso_tp.h"
#include "uds_client.h"
#include "vw_mqb.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <net/if.h>          // if_nametoindex()
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace {

struct Args {
    std::string iface         = "can0";
    uint32_t    tx_id         = 0x18DA10F1;
    uint32_t    rx_id         = 0x18DAF110;
    bool        apply         = false;    // safe default: dry-run
    uint8_t     sec_sub       = vw::DEFAULT_SEC_REQ_SEED_SUB;
    uint32_t    static_login  = 20103;    // legacy VW Gateway login -- ymmv
    bool        quiet         = false;
    std::string backup_file;              // required for --apply
    bool        no_backup     = false;    // escape hatch (dev only)

    std::string              cmd;
    std::vector<std::string> rest;
};

[[noreturn]] void die(const std::string& msg) {
    std::cerr << "Error: " << msg << std::endl;
    std::exit(1);
}

uint32_t parseHexU32(const std::string& s) {
    return static_cast<uint32_t>(std::stoul(s, nullptr, 16));
}

Args parseArgs(int argc, char** argv) {
    Args a;
    int i = 1;
    for (; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&](int n) {
            if (i + n >= argc) die("missing value after " + k);
        };
        if      (k == "--iface")        { need(1); a.iface        = argv[++i]; }
        else if (k == "--tx")           { need(1); a.tx_id        = parseHexU32(argv[++i]); }
        else if (k == "--rx")           { need(1); a.rx_id        = parseHexU32(argv[++i]); }
        else if (k == "--apply")        { a.apply = true; }
        else if (k == "--sec-sub")      { need(1); a.sec_sub      = static_cast<uint8_t>(parseHexU32(argv[++i])); }
        else if (k == "--static-login") { need(1); a.static_login = static_cast<uint32_t>(std::stoul(argv[++i])); }
        else if (k == "--backup")       { need(1); a.backup_file   = argv[++i]; }
        else if (k == "--no-backup")    { a.no_backup = true; }
        else if (k == "--quiet")        { a.quiet = true; }
        else if (k == "--help" || k == "-h") {
            std::cout <<
                "Usage: uds_main [options] <command> [args]\n"
                "  --iface <name>        CAN interface (default can0)\n"
                "  --tx <HEX>            Tester-to-ECU 29-bit ID (default 18DA10F1)\n"
                "  --rx <HEX>            ECU-to-tester 29-bit ID (default 18DAF110)\n"
                "  --apply               Actually write. Without this flag, all\n"
                "                        write commands run in dry-run mode.\n"
                "  --sec-sub <HEX>       Security Access request-seed sub-function\n"
                "                        (odd; default 03)\n"
                "  --static-login <DEC>  Legacy VW login code (default 20103)\n"
                "  --backup <FILE>       Required with --apply: backup file\n"
                "                        whose fingerprint must match live ECU\n"
                "  --no-backup           Dev-only override for --apply\n"
                "  --quiet               Disable per-frame hex logging\n"
                "\nCommands:\n"
                "  ident\n"
                "  self-test\n"
                "  read-did <DIDHEX>\n"
                "  write-did <DIDHEX> <BYTES>\n"
                "  dump-list\n"
                "  dump-coding\n"
                "  fix-ecall\n"
                "  disable-ss\n"
                "  backup  <FILE>\n"
                "  restore <FILE>\n";
            std::exit(0);
        }
        else { break; }
    }
    if (i >= argc) die("no command given (try --help)");
    a.cmd = argv[i++];
    for (; i < argc; ++i) a.rest.emplace_back(argv[i]);
    return a;
}

// Open a SocketCAN raw socket, bind to `iface`, install a filter that only
// accepts frames with CAN id == rx_id (extended). This removes the need for
// manual filtering on the read path and drops the chatty Gateway broadcast
// the user was already ignoring in advanced_smart_reader.cpp.
int openCanSocket(const std::string& iface, uint32_t rx_id, bool extended) {
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) die(std::string("socket: ") + std::strerror(errno));

    unsigned int ifindex = ::if_nametoindex(iface.c_str());
    if (ifindex == 0) {
        ::close(s);
        die(std::string("if_nametoindex(") + iface + "): " + std::strerror(errno));
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = static_cast<int>(ifindex);
    if (::bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(s);
        die(std::string("bind: ") + std::strerror(errno));
    }

    struct can_filter flt[1];
    flt[0].can_id   = rx_id | (extended ? CAN_EFF_FLAG : 0);
    flt[0].can_mask = (extended ? (CAN_EFF_MASK | CAN_EFF_FLAG) : CAN_SFF_MASK);
    if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt)) < 0) {
        ::close(s);
        die(std::string("setsockopt CAN_RAW_FILTER: ") + std::strerror(errno));
    }
    return s;
}

// Load + verify a backup file against the live ECU. Returns true if either:
//   - we're in dry-run (no verification needed), OR
//   - apply mode AND a valid backup was supplied AND the fingerprint matches,
//     OR --no-backup override was passed.
// Returns false only if the user asked to apply but their backup disagrees
// with the car -- in which case we MUST refuse to write.
bool enforceBackupPolicy(const Args& a, uds::UdsClient& client) {
    if (!a.apply) return true;                    // dry-run is always safe
    if (a.no_backup) {
        std::cerr << "[policy] --no-backup override: proceeding without a backup.\n"
                     "         This is intended for development only.\n";
        return true;
    }
    if (a.backup_file.empty()) {
        std::cerr << "[policy] --apply requires --backup <FILE>. Refusing to write "
                     "without a verified snapshot.\n"
                     "         Run `uds_main backup pre-change.txt` first.\n";
        return false;
    }
    backup::Data b;
    if (!backup::load(a.backup_file, b)) return false;
    backup::print(b);
    bool sw_changed = false;
    if (!backup::verifyIdentity(client, b, sw_changed)) {
        std::cerr << "[policy] Backup does not match this Gateway. Refusing to write.\n";
        return false;
    }
    std::cout << "[policy] Backup fingerprint matches live ECU. OK to proceed.\n";
    return true;
}

void printIdent(const vw::GatewayIdentity& id) {
    std::cout << "  VIN            : " << id.vin            << "\n"
              << "  Part Number    : " << id.part_number    << "\n"
              << "  SW Version     : " << id.sw_version     << "\n"
              << "  Serial Number  : " << id.serial_number  << "\n";
}

// ---- Command implementations -------------------------------------------------

int cmdIdent(uds::UdsClient& c) {
    auto r = c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!r.ok) { std::cerr << "Enter Extended Session failed: " << r.describe() << "\n"; return 1; }

    vw::GatewayIdentity id;
    if (!vw::readGatewayIdentity(c, id)) {
        std::cerr << "Failed to read identity.\n";
        return 1;
    }
    std::cout << "Gateway identity:\n";
    printIdent(id);
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return 0;
}

int cmdSelfTest(uds::UdsClient& c) {
    auto r = c.diagnosticSessionControl(uds::Session::EXTENDED);
    std::cout << "ExtendedSession: " << r.describe() << "\n";
    auto t = c.testerPresent(false);
    std::cout << "TesterPresent:   " << t.describe() << "\n";
    auto d = c.diagnosticSessionControl(uds::Session::DEFAULT);
    std::cout << "DefaultSession:  " << d.describe() << "\n";
    return (r.ok && t.ok && d.ok) ? 0 : 1;
}

int cmdReadDid(uds::UdsClient& c, const std::vector<std::string>& rest) {
    if (rest.size() != 1) die("read-did needs exactly one <DIDHEX> argument");
    uint16_t did = static_cast<uint16_t>(parseHexU32(rest[0]));
    c.diagnosticSessionControl(uds::Session::EXTENDED);
    auto r = c.readDataByIdentifier(did);
    if (!r.ok) { std::cerr << "RDBI failed: " << r.describe() << "\n"; return 1; }
    std::cout << "DID 0x" << std::hex << did << ":\n  "
              << hexu::bytesToHex(r.data) << "\n";
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return 0;
}

int cmdWriteDid(uds::UdsClient& c, const Args& a) {
    if (a.rest.size() < 2) die("write-did needs <DIDHEX> <BYTES>");
    uint16_t did = static_cast<uint16_t>(parseHexU32(a.rest[0]));

    std::vector<uint8_t> payload;
    if (!hexu::parseHexString(a.rest[1], payload)) die("malformed <BYTES>");

    if (!a.apply) {
        std::cout << "DRY-RUN: would write DID 0x" << std::hex << did << " <- "
                  << hexu::bytesToHex(payload) << "\n"
                  << "Re-run with --apply --backup <file> to actually write.\n";
        return 0;
    }

    c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!enforceBackupPolicy(a, c)) { c.diagnosticSessionControl(uds::Session::DEFAULT); return 2; }

    auto sa = c.securityAccess(a.sec_sub, [&](const std::vector<uint8_t>& seed) {
        (void)seed;
        return vw::keyFromStaticLogin(a.static_login);
    });
    if (!sa.ok) { std::cerr << "SecurityAccess failed: " << sa.describe() << "\n"; return 1; }

    auto w = c.writeDataByIdentifier(did, payload);
    std::cout << "WDBI: " << w.describe() << "\n";
    if (w.ok) {
        auto rb = c.readDataByIdentifier(did);
        if (!rb.ok || rb.data.size() < 2
            || std::vector<uint8_t>(rb.data.begin()+2, rb.data.end()) != payload) {
            std::cerr << "WDBI readback did NOT match what we wrote. Aborting.\n";
            c.diagnosticSessionControl(uds::Session::DEFAULT);
            return 1;
        }
        std::cout << "WDBI readback verified.\n";
    }
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return w.ok ? 0 : 1;
}

int cmdDumpList(uds::UdsClient& c) {
    c.diagnosticSessionControl(uds::Session::EXTENDED);
    vw::InstallationList list;
    if (!vw::readInstallationList(c, list)) return 1;
    vw::printInstallationList(list, std::cout);
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return 0;
}

int cmdDumpCoding(uds::UdsClient& c) {
    c.diagnosticSessionControl(uds::Session::EXTENDED);
    auto r = c.readDataByIdentifier(vw::DID::VW_CODING_VALUE);
    if (!r.ok) { std::cerr << "RDBI 0x0600 failed: " << r.describe() << "\n"; return 1; }
    std::cout << "Coding (" << r.data.size() << " bytes, first 2 are DID echo):\n  "
              << hexu::bytesToHex(r.data) << "\n";
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return 0;
}

int cmdFixEcall(uds::UdsClient& c, const Args& a) {
    auto ext = c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!ext.ok) { std::cerr << "ExtendedSession failed: " << ext.describe() << "\n"; return 1; }

    if (!enforceBackupPolicy(a, c)) { c.diagnosticSessionControl(uds::Session::DEFAULT); return 2; }

    if (a.apply) {
        auto sa = c.securityAccess(a.sec_sub, [&](const std::vector<uint8_t>& seed) {
            (void)seed;
            return vw::keyFromStaticLogin(a.static_login);
        });
        if (!sa.ok) {
            std::cerr << "SecurityAccess failed: " << sa.describe() << "\n"
                      << "Your Gateway likely requires a real seed/key algorithm.\n"
                      << "Plug a proper KeyFn into securityAccess(...) and retry.\n";
            return 1;
        }
    } else {
        std::cout << "DRY-RUN: skipping SecurityAccess.\n";
    }

    bool ok = vw::fixEmergencyCall(c, /*dry_run=*/!a.apply);
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return ok ? 0 : 1;
}

int cmdDisableSs(uds::UdsClient& c, const Args& a) {
    auto ext = c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!ext.ok) { std::cerr << "ExtendedSession failed: " << ext.describe() << "\n"; return 1; }

    if (!enforceBackupPolicy(a, c)) { c.diagnosticSessionControl(uds::Session::DEFAULT); return 2; }

    if (a.apply) {
        auto sa = c.securityAccess(a.sec_sub, [&](const std::vector<uint8_t>& seed) {
            (void)seed;
            return vw::keyFromStaticLogin(a.static_login);
        });
        if (!sa.ok) {
            std::cerr << "SecurityAccess failed: " << sa.describe() << "\n";
            return 1;
        }
    } else {
        std::cout << "DRY-RUN: skipping SecurityAccess.\n";
    }

    bool ok = vw::disableStartStop(c, /*dry_run=*/!a.apply);
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return ok ? 0 : 1;
}

int cmdBackup(uds::UdsClient& c, const std::vector<std::string>& rest) {
    if (rest.size() != 1) die("backup needs <FILE>");
    auto ext = c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!ext.ok) { std::cerr << "ExtendedSession failed: " << ext.describe() << "\n"; return 1; }

    backup::Data d;
    if (!backup::capture(c, d)) {
        c.diagnosticSessionControl(uds::Session::DEFAULT);
        return 1;
    }
    if (!backup::save(rest[0], d)) {
        std::cerr << "Failed to write backup to " << rest[0] << "\n";
        c.diagnosticSessionControl(uds::Session::DEFAULT);
        return 1;
    }
    backup::print(d);
    std::cout << "Backup saved to " << rest[0] << "\n";
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return 0;
}

int cmdRestore(uds::UdsClient& c, const Args& a) {
    if (a.rest.size() != 1) die("restore needs <FILE>");
    auto ext = c.diagnosticSessionControl(uds::Session::EXTENDED);
    if (!ext.ok) { std::cerr << "ExtendedSession failed: " << ext.describe() << "\n"; return 1; }

    backup::Data d;
    if (!backup::load(a.rest[0], d)) {
        c.diagnosticSessionControl(uds::Session::DEFAULT);
        return 1;
    }
    backup::print(d);

    if (a.apply) {
        auto sa = c.securityAccess(a.sec_sub, [&](const std::vector<uint8_t>& seed) {
            (void)seed;
            return vw::keyFromStaticLogin(a.static_login);
        });
        if (!sa.ok) {
            std::cerr << "SecurityAccess failed: " << sa.describe() << "\n";
            c.diagnosticSessionControl(uds::Session::DEFAULT);
            return 1;
        }
    } else {
        std::cout << "DRY-RUN: skipping SecurityAccess.\n";
    }

    bool ok = backup::restore(c, d, /*dry_run=*/!a.apply);
    c.diagnosticSessionControl(uds::Session::DEFAULT);
    return ok ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    Args a = parseArgs(argc, argv);

    int s = openCanSocket(a.iface, a.rx_id, /*extended=*/true);

    isotp::Config cfg;
    cfg.tx_id       = a.tx_id;
    cfg.rx_id       = a.rx_id;
    cfg.extended_id = true;
    cfg.verbose     = !a.quiet;
    cfg.timeout_ms  = 1500;

    uds::UdsClient c(s, cfg);

    if (!a.apply) {
        std::cout << "==========================================================\n"
                     " DRY-RUN MODE -- no writes will be sent. Use --apply\n"
                     " once you have verified the planned changes are correct.\n"
                     "==========================================================\n";
    } else {
        std::cout << "==========================================================\n"
                     " APPLY MODE -- writes WILL be sent to the Gateway.\n"
                     " Engine off, battery on charger, no other diag tool active.\n"
                     "==========================================================\n";
    }

    int rc = 1;
    if      (a.cmd == "ident")        rc = cmdIdent(c);
    else if (a.cmd == "self-test")    rc = cmdSelfTest(c);
    else if (a.cmd == "read-did")     rc = cmdReadDid(c, a.rest);
    else if (a.cmd == "write-did")    rc = cmdWriteDid(c, a);
    else if (a.cmd == "dump-list")    rc = cmdDumpList(c);
    else if (a.cmd == "dump-coding")  rc = cmdDumpCoding(c);
    else if (a.cmd == "fix-ecall")    rc = cmdFixEcall(c, a);
    else if (a.cmd == "disable-ss")   rc = cmdDisableSs(c, a);
    else if (a.cmd == "backup")       rc = cmdBackup(c, a.rest);
    else if (a.cmd == "restore")      rc = cmdRestore(c, a);
    else { std::cerr << "Unknown command: " << a.cmd << "\n"; rc = 2; }

    ::close(s);
    return rc;
}
