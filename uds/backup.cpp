#include "backup.h"
#include "hex_utils.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

namespace backup {

static std::string isoNowUtc() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buf);
}

static std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool capture(uds::UdsClient& client, Data& out) {
    out = Data{};
    out.timestamp = isoNowUtc();

    vw::GatewayIdentity id;
    if (!vw::readGatewayIdentity(client, id)) {
        std::cerr << "[backup] readGatewayIdentity failed.\n";
        return false;
    }
    out.vin          = id.vin;
    out.part_number  = id.part_number;
    out.sw_version   = id.sw_version;
    out.serial       = id.serial_number;

    // Coding (DID 0x0600).
    auto r = client.readDataByIdentifier(vw::DID::VW_CODING_VALUE);
    if (!r.ok) {
        std::cerr << "[backup] RDBI 0x0600 failed: " << r.describe() << "\n";
        return false;
    }
    if (r.data.size() < 2) {
        std::cerr << "[backup] Coding response too short.\n";
        return false;
    }
    out.coding.assign(r.data.begin() + 2, r.data.end());

    // Installation List (DID 0x0608).
    vw::InstallationList list;
    if (!vw::readInstallationList(client, list)) return false;
    out.installation_list = list.raw;

    return true;
}

bool save(const std::string& path, const Data& d) {
    std::ofstream f(path, std::ios::trunc);
    if (!f) return false;
    f << "# ELM-UDS backup v1\n";
    f << "vin="          << d.vin          << "\n";
    f << "part_number="  << d.part_number  << "\n";
    f << "sw_version="   << d.sw_version   << "\n";
    f << "serial="       << d.serial       << "\n";
    f << "timestamp="    << d.timestamp    << "\n";
    f << "coding_did=0600\n";
    f << "coding="              << hexu::bytesToHex(d.coding) << "\n";
    f << "installation_list_did=0608\n";
    f << "installation_list="   << hexu::bytesToHex(d.installation_list) << "\n";
    return f.good();
}

bool load(const std::string& path, Data& out) {
    out = Data{};
    std::ifstream f(path);
    if (!f) {
        std::cerr << "[backup] cannot open " << path << "\n";
        return false;
    }
    std::string line;
    while (std::getline(f, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;
        auto pos = t.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(t.substr(0, pos));
        std::string val = trim(t.substr(pos + 1));
        if      (key == "vin")               out.vin         = val;
        else if (key == "part_number")       out.part_number = val;
        else if (key == "sw_version")        out.sw_version  = val;
        else if (key == "serial")            out.serial      = val;
        else if (key == "timestamp")         out.timestamp   = val;
        else if (key == "coding") {
            if (!hexu::parseHexString(val, out.coding)) {
                std::cerr << "[backup] bad coding hex\n"; return false;
            }
        }
        else if (key == "installation_list") {
            if (!hexu::parseHexString(val, out.installation_list)) {
                std::cerr << "[backup] bad installation_list hex\n"; return false;
            }
        }
    }
    if (out.vin.empty() || out.coding.empty() || out.installation_list.empty()) {
        std::cerr << "[backup] file missing required fields\n";
        return false;
    }
    return true;
}

bool verifyIdentity(uds::UdsClient& client, const Data& b,
                    bool& sw_version_changed) {
    sw_version_changed = false;
    vw::GatewayIdentity id;
    if (!vw::readGatewayIdentity(client, id)) {
        std::cerr << "[backup] identity read failed.\n";
        return false;
    }
    if (id.vin != b.vin) {
        std::cerr << "[backup] VIN MISMATCH! backup=" << b.vin
                  << "  car=" << id.vin << "\n";
        return false;
    }
    if (id.part_number != b.part_number) {
        std::cerr << "[backup] Part-number MISMATCH! backup=" << b.part_number
                  << "  car=" << id.part_number
                  << "\n  (different hardware means different coding layout; "
                     "do NOT restore this backup to this ECU.)\n";
        return false;
    }
    if (id.sw_version != b.sw_version) {
        std::cerr << "[backup] WARNING: SW version changed ("
                  << b.sw_version << " -> " << id.sw_version
                  << "). Restore will still proceed, but double-check the "
                     "backup is still valid.\n";
        sw_version_changed = true;
    }
    return true;
}

bool restore(uds::UdsClient& client, const Data& b, bool dry_run) {
    bool sw_changed = false;
    if (!verifyIdentity(client, b, sw_changed)) return false;

    std::cout << "[restore] Will write coding (" << b.coding.size()
              << " bytes) and installation list ("
              << b.installation_list.size() << " bytes) back to DIDs "
              << "0x0600 and 0x0608.\n";

    if (dry_run) {
        std::cout << "[restore] DRY-RUN: not sending writes.\n";
        return true;
    }

    // Coding first.
    auto w1 = client.writeDataByIdentifier(vw::DID::VW_CODING_VALUE, b.coding);
    if (!w1.ok) {
        std::cerr << "[restore] WriteDataByIdentifier 0x0600 failed: "
                  << w1.describe() << "\n";
        return false;
    }
    // Readback verify.
    auto r1 = client.readDataByIdentifier(vw::DID::VW_CODING_VALUE);
    if (!r1.ok || r1.data.size() < 2
        || std::vector<uint8_t>(r1.data.begin()+2, r1.data.end()) != b.coding) {
        std::cerr << "[restore] Readback of 0x0600 did not match what we wrote!\n";
        return false;
    }
    std::cout << "[restore] 0x0600 write verified.\n";

    auto w2 = client.writeDataByIdentifier(vw::DID::VW_INSTALLATION_LIST,
                                           b.installation_list);
    if (!w2.ok) {
        std::cerr << "[restore] WriteDataByIdentifier 0x0608 failed: "
                  << w2.describe() << "\n";
        return false;
    }
    auto r2 = client.readDataByIdentifier(vw::DID::VW_INSTALLATION_LIST);
    if (!r2.ok || r2.data.size() < 2
        || std::vector<uint8_t>(r2.data.begin()+2, r2.data.end()) != b.installation_list) {
        std::cerr << "[restore] Readback of 0x0608 did not match what we wrote!\n";
        return false;
    }
    std::cout << "[restore] 0x0608 write verified. Restore complete.\n";
    return true;
}

void print(const Data& d) {
    std::cout << "Backup contents:\n"
              << "  VIN          : " << d.vin          << "\n"
              << "  Part Number  : " << d.part_number  << "\n"
              << "  SW Version   : " << d.sw_version   << "\n"
              << "  Serial       : " << d.serial       << "\n"
              << "  Timestamp    : " << d.timestamp    << "\n"
              << "  coding       : " << d.coding.size() << " bytes\n"
              << "  install list : " << d.installation_list.size() << " bytes\n";
}

} // namespace backup
