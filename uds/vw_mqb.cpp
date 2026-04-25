#include "vw_mqb.h"
#include "hex_utils.h"

#include <cstddef>
#include <iomanip>
#include <iostream>

namespace vw {

// ---------------------------------------------------------------------------
// Seed-key helpers
// ---------------------------------------------------------------------------
std::vector<uint8_t> keyFromStaticLogin(uint32_t login_code) {
    return {
        static_cast<uint8_t>((login_code >> 24) & 0xFF),
        static_cast<uint8_t>((login_code >> 16) & 0xFF),
        static_cast<uint8_t>((login_code >>  8) & 0xFF),
        static_cast<uint8_t>( login_code        & 0xFF),
    };
}

std::vector<uint8_t> keyFromSeedPlaceholder(const std::vector<uint8_t>& seed) {
    std::vector<uint8_t> k = seed;
    for (auto& b : k) b ^= 0xFF;
    return k;
}

// ---------------------------------------------------------------------------
// InstallationList: the Gateway's "is module X present on the bus?" bitmap.
//
// The raw data is a vector of up to 256 bytes. Think of it as a dict-like
// structure where the *index* of the byte is the ECU address:
//
//     raw[0x01]  -> Engine  (ECU 01)
//     raw[0x17]  -> Cluster (ECU 17)
//     raw[0x19]  -> Gateway (ECU 19)
//     raw[0x75]  -> Telematics / eCall module (ECU 75) <-- the one we care about
//
// Each byte has multiple bits in it, but bit 0 ("the lowest bit") means
// "this module is installed in the car". The other bits encode coding
// variant / subnet / protocol version and we must NOT touch them.
//
// Python analogy -- if `raw` were a `bytearray`:
//     raw[0x75] & 0x01   ==  is Module 75 installed?
//     raw[0x75] |= 0x01  ==  mark installed (set bit 0)
//     raw[0x75] &= 0xFE  ==  mark not installed (clear bit 0; ~0x01 == 0xFE)
// ---------------------------------------------------------------------------
bool InstallationList::isInstalled(uint8_t addr) const {
    if (addr >= raw.size()) return false;
    // `raw[addr] & 0x01` isolates the lowest bit. If it was 1, the `!= 0`
    // comparison returns true.
    return (raw[addr] & 0x01) != 0;
}

bool InstallationList::setInstalled(uint8_t addr, bool installed) {
    if (addr >= raw.size()) return false;
    if (installed) {
        // Set bit 0:   byte OR-ed with 00000001 -> lowest bit becomes 1,
        //              every other bit unchanged.
        raw[addr] |=  0x01;
    } else {
        // Clear bit 0: byte AND-ed with 11111110 (= ~0x01) -> lowest bit
        //              becomes 0, every other bit unchanged. This is THE
        //              single operation that turns off the SOS warning.
        raw[addr] &= ~0x01;
    }
    return true;
}

void printInstallationList(const InstallationList& list, std::ostream& os) {
    os << "Installation List (" << list.raw.size() << " bytes):\n";
    for (size_t i = 0; i < list.raw.size(); ++i) {
        uint8_t b = list.raw[i];
        bool installed = (b & 0x01) != 0;
        os << "  addr 0x" << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(i)
           << " : 0x" << std::setw(2) << static_cast<int>(b)
           << (installed ? "   [INSTALLED]" : "")
           << std::dec << std::setfill(' ') << '\n';
    }
}

// ---------------------------------------------------------------------------
// High-level helpers
// ---------------------------------------------------------------------------
static std::string asAscii(const std::vector<uint8_t>& v) {
    std::string s;
    s.reserve(v.size());
    for (uint8_t b : v) {
        s.push_back((b >= 0x20 && b < 0x7F) ? static_cast<char>(b) : '.');
    }
    return s;
}

bool readGatewayIdentity(uds::UdsClient& client, GatewayIdentity& out) {
    auto r1 = client.readDataByIdentifier(DID::VEHICLE_IDENTIFICATION_NUMBER);
    // DID echo is the first 2 bytes of a positive RDBI response; strip them.
    if (r1.ok && r1.data.size() > 2) out.vin = asAscii({r1.data.begin()+2, r1.data.end()});

    auto r2 = client.readDataByIdentifier(DID::VW_SPARE_PART_NUMBER);
    if (r2.ok && r2.data.size() > 2) out.part_number = asAscii({r2.data.begin()+2, r2.data.end()});

    auto r3 = client.readDataByIdentifier(DID::VW_APPLICATION_SW_VERSION);
    if (r3.ok && r3.data.size() > 2) out.sw_version = asAscii({r3.data.begin()+2, r3.data.end()});

    auto r4 = client.readDataByIdentifier(DID::VW_ECU_SERIAL_NUMBER);
    if (r4.ok && r4.data.size() > 2) out.serial_number = asAscii({r4.data.begin()+2, r4.data.end()});

    return r1.ok || r2.ok;   // at least VIN or part number must come back
}

bool readInstallationList(uds::UdsClient& client, InstallationList& out) {
    auto r = client.readDataByIdentifier(DID::VW_INSTALLATION_LIST);
    if (!r.ok) {
        std::cerr << "[VW] Read Installation List failed: " << r.describe() << std::endl;
        return false;
    }
    // Strip the 2-byte DID echo from the front of the RDBI payload.
    if (r.data.size() < 2) {
        std::cerr << "[VW] Installation List response too short." << std::endl;
        return false;
    }
    out.raw.assign(r.data.begin() + 2, r.data.end());
    return true;
}

// Write Installation List with mandatory readback-verify:
//   1. Write the buffer via 0x2E.
//   2. Read it back via 0x22.
//   3. Compare byte-for-byte. If different, SCREAM and return false.
// This is the single best insurance policy against writes that "look OK" at
// the protocol layer but actually land as garbage in EEPROM.
bool writeInstallationList(uds::UdsClient& client,
                           const InstallationList& list,
                           bool dry_run) {
    std::cout << "[VW] Installation List to write ("
              << list.raw.size() << " bytes):\n  "
              << hexu::bytesToHex(list.raw) << std::endl;

    if (dry_run) {
        std::cout << "[VW] DRY-RUN: not sending WriteDataByIdentifier." << std::endl;
        return true;
    }

    auto w = client.writeDataByIdentifier(DID::VW_INSTALLATION_LIST, list.raw);
    if (!w.ok) {
        std::cerr << "[VW] Write Installation List failed: " << w.describe() << std::endl;
        return false;
    }
    std::cout << "[VW] Installation List write OK. Verifying via readback..." << std::endl;

    auto r = client.readDataByIdentifier(DID::VW_INSTALLATION_LIST);
    if (!r.ok || r.data.size() < 2) {
        std::cerr << "[VW] Readback failed: " << r.describe() << std::endl;
        return false;
    }
    std::vector<uint8_t> got(r.data.begin() + 2, r.data.end());
    if (got != list.raw) {
        std::cerr << "[VW] READBACK MISMATCH. The Gateway accepted our write "
                     "but the stored value is different!\n"
                     "  wrote : " << hexu::bytesToHex(list.raw) << "\n"
                     "  read  : " << hexu::bytesToHex(got) << std::endl;
        return false;
    }
    std::cout << "[VW] Installation List readback matches write. OK." << std::endl;
    return true;
}

bool fixEmergencyCall(uds::UdsClient& client, bool dry_run) {
    InstallationList before;
    if (!readInstallationList(client, before)) return false;

    std::cout << "[VW] --- BEFORE ---" << std::endl;
    printInstallationList(before, std::cout);

    constexpr uint8_t MOD_TELEMATICS = 0x75;   // Module 75 (OCU / eCall)
    if (before.raw.size() <= MOD_TELEMATICS) {
        std::cerr << "[VW] Installation List is shorter than 0x76 bytes ("
                  << before.raw.size() << " bytes) -- this Gateway layout is "
                     "different from what this helper assumes. Aborting to "
                     "avoid writing garbage."
                  << std::endl;
        return false;
    }
    if (!before.isInstalled(MOD_TELEMATICS)) {
        std::cout << "[VW] Module 0x75 (Telematics) is already marked as NOT "
                     "installed -- nothing to do."
                  << std::endl;
        return true;
    }

    InstallationList after = before;
    after.setInstalled(MOD_TELEMATICS, false);

    std::cout << "[VW] --- AFTER (planned) ---" << std::endl;
    printInstallationList(after, std::cout);

    std::cout << "[VW] Byte 0x75 change: 0x"
              << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(before.raw[MOD_TELEMATICS])
              << " -> 0x"
              << std::setw(2) << static_cast<int>(after.raw[MOD_TELEMATICS])
              << std::dec << std::setfill(' ') << std::endl;

    return writeInstallationList(client, after, dry_run);
}

// ---------------------------------------------------------------------------
// disableStartStop: flip one bit inside the Gateway's long coding.
//
// Unlike the Installation List, coding bytes are NOT one-bit-per-module. A
// single coding byte typically encodes several unrelated feature flags
// packed together. That means:
//
//   - We MUST read the whole coding buffer first, modify ONE byte, and
//     write the exact same buffer back. Writing a shorter/zeroed buffer
//     would wipe out every feature at once (Start-Stop *and* parking
//     assist *and* trailer mode etc.).
//   - The exact byte index and bit mask for Start-Stop vary by Gateway
//     hardware + software. Those two values live in a VCDS label file.
//     We take them as parameters so the caller can supply the correct
//     pair for their specific car.
// ---------------------------------------------------------------------------
bool disableStartStop(uds::UdsClient& client, bool dry_run,
                      size_t coding_byte_index,
                      uint8_t bit_mask,
                      bool    clear_bit_to_disable) {
    // Step 1: read the current coding buffer.
    auto r = client.readDataByIdentifier(DID::VW_CODING_VALUE);
    if (!r.ok) {
        std::cerr << "[VW] Read Coding (DID 0x0600) failed: "
                  << r.describe() << std::endl;
        return false;
    }
    if (r.data.size() < 2) {
        std::cerr << "[VW] Coding response too short." << std::endl;
        return false;
    }
    // `r.data` starts with the echoed 2-byte DID (0x06, 0x00). Strip it to
    // get the actual coding payload.
    std::vector<uint8_t> coding(r.data.begin() + 2, r.data.end());

    std::cout << "[VW] Current coding (" << coding.size() << " bytes):\n  "
              << hexu::bytesToHex(coding) << std::endl;

    if (coding_byte_index >= coding.size()) {
        std::cerr << "[VW] coding_byte_index " << coding_byte_index
                  << " is out of range (" << coding.size() << " bytes). "
                  << "Check the VCDS label file for your Gateway part number "
                  << "and re-run with the correct index." << std::endl;
        return false;
    }

    // Step 2: compute the new byte value.
    //
    // `bit_mask` says WHICH bits we care about. Typical values are a single
    // bit (e.g. 0x20 = 0b00100000) or sometimes a pair.
    //
    //   clear_bit_to_disable == true
    //     after = before & ~mask
    //           = "keep every bit of `before`, except the bits in `mask`,
    //              which are forced to 0"
    //     In JS:   after = before & ~mask
    //
    //   clear_bit_to_disable == false
    //     after = before | mask
    //           = "keep every bit of `before`, and additionally set any bit
    //              in `mask`"
    //     In JS:   after = before | mask
    uint8_t before = coding[coding_byte_index];
    uint8_t after  = clear_bit_to_disable
                   ? static_cast<uint8_t>(before & ~bit_mask)
                   : static_cast<uint8_t>(before |  bit_mask);

    std::cout << "[VW] Byte[" << coding_byte_index << "]: 0x"
              << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(before) << " -> 0x"
              << std::setw(2) << static_cast<int>(after)
              << std::dec << std::setfill(' ')
              << "  (mask=0x" << std::hex << static_cast<int>(bit_mask)
              << ", " << (clear_bit_to_disable ? "clear" : "set") << ")"
              << std::dec << std::endl;

    if (before == after) {
        std::cout << "[VW] Coding already in the requested state -- nothing to do."
                  << std::endl;
        return true;
    }
    coding[coding_byte_index] = after;

    if (dry_run) {
        std::cout << "[VW] DRY-RUN: would write coding:\n  "
                  << hexu::bytesToHex(coding) << std::endl;
        return true;
    }

    auto w = client.writeDataByIdentifier(DID::VW_CODING_VALUE, coding);
    if (!w.ok) {
        std::cerr << "[VW] Write Coding (DID 0x0600) failed: "
                  << w.describe() << std::endl;
        return false;
    }

    // Mandatory readback verify. The ECU may ACK a write but still store
    // something different (wrong length, checksum reject, race with BCM
    // validation, etc). We only return success if what we read back is
    // byte-for-byte identical to what we wrote.
    auto rb = client.readDataByIdentifier(DID::VW_CODING_VALUE);
    if (!rb.ok || rb.data.size() < 2) {
        std::cerr << "[VW] Coding readback failed: " << rb.describe() << std::endl;
        return false;
    }
    std::vector<uint8_t> got(rb.data.begin() + 2, rb.data.end());
    if (got != coding) {
        std::cerr << "[VW] CODING READBACK MISMATCH. The Gateway accepted our "
                     "write but the stored value differs!\n"
                     "  wrote : " << hexu::bytesToHex(coding) << "\n"
                     "  read  : " << hexu::bytesToHex(got) << std::endl;
        return false;
    }
    std::cout << "[VW] Coding write verified (readback matches). Start-Stop disabled." << std::endl;
    return true;
}

} // namespace vw
