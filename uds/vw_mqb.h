#pragma once

// Volkswagen / MQB-specific helpers on top of the generic UdsClient.
//
// WARNING
// -------
// The DIDs, security-access level and key algorithm below are *generic*
// placeholders that cover the shapes most commonly documented for MQB
// Gateways (J533 / ECU 0x19). The *exact* values vary by:
//   - Gateway hardware / part number (J533 vs J533 Plus vs J794 etc.)
//   - Software version
//   - Market (NAR/RoW/China) and model year
//
// Before writing anything:
//   1. Run `read-did` against F187 (part number) and F189 (SW version) to
//      identify your exact Gateway.
//   2. Run `dump-coding` to capture and save the full current coding /
//      installation list to a file -- this is your rollback image.
//   3. Cross-check the specific adaptation against your vehicle's coding
//      documentation (VCDS label file, ODIS trace, Erwin, etc.).

#include "uds_client.h"

#include <cstdint>
#include <string>
#include <vector>

namespace vw {

// ---------------------------------------------------------------------------
// Standardised VW DIDs (these are stable across the MQB lineup).
// ---------------------------------------------------------------------------
namespace DID {
constexpr uint16_t ACTIVE_DIAGNOSTIC_SESSION          = 0xF186;
constexpr uint16_t VW_SPARE_PART_NUMBER               = 0xF187;
constexpr uint16_t VW_ECU_HARDWARE_NUMBER             = 0xF191;
constexpr uint16_t VW_APPLICATION_SW_VERSION          = 0xF189;
constexpr uint16_t VW_ECU_SERIAL_NUMBER               = 0xF18C;
constexpr uint16_t VEHICLE_IDENTIFICATION_NUMBER      = 0xF190;
constexpr uint16_t SYSTEM_NAME_OR_ENGINE_TYPE         = 0xF197;
constexpr uint16_t VW_CODING_VALUE                    = 0x0600;  // long coding
constexpr uint16_t VW_INSTALLATION_LIST               = 0x0608;  // subsystem/installation bitmap
} // namespace DID

// VW-specific "login" style adaptations typically live in the extended
// session behind Security Access. The default level observed on most MQB
// Gateways for coding/adaptation is sub-function 0x03 (request seed) /
// 0x04 (send key). If your trace shows 0x0B/0x0C or 0x11/0x12 instead, set
// REQ_SEED_SUB accordingly in main.cpp.
constexpr uint8_t DEFAULT_SEC_REQ_SEED_SUB = 0x03;

// ---------------------------------------------------------------------------
// Seed -> key transformation.
//
// For VW MQB gateways the real algorithm is proprietary and depends on the
// Gateway's software version. There is NO portable in-the-clear implementation
// that will work across all J533 variants. You have three realistic options:
//
//   (a) Capture an ODIS/VCDS session with Wireshark + SocketCAN, recover the
//       seed/key pair(s), and reverse the algorithm yourself.
//   (b) Feed the seed to an external service (a pre-computed lookup table or
//       your own C++ implementation of the recovered algorithm) and plug the
//       result back into `key_fn`.
//   (c) On very old/unlocked Gateway software variants only, a static
//       KWP2000-style "login code" (e.g. 20103, 31347) used to be accepted;
//       this is not standard UDS Security Access and will return NRC 0x35
//       (invalidKey) or 0x33 (securityAccessDenied) on a locked modern ECU.
//
// `keyFromStaticLogin` implements option (c) as a best-effort for people who
// know their specific car accepts it. It simply packs the 5-digit decimal
// login code into a 4-byte big-endian key and returns it.
// ---------------------------------------------------------------------------
std::vector<uint8_t> keyFromStaticLogin(uint32_t login_code);

// Placeholder: XOR each seed byte with 0xFF. DOES NOT WORK on real hardware.
// Kept here so the UI flow is exercisable in a dry/virtual CAN setup.
std::vector<uint8_t> keyFromSeedPlaceholder(const std::vector<uint8_t>& seed);

// ---------------------------------------------------------------------------
// Installation List (Verbauliste) helpers.
//
// The Installation List is a bitmap returned by the Gateway that tells the
// car which subsystems are *supposed to be present* on the diagnostic bus.
// On most MQB gateways the list is a sequence of bytes where each byte
// represents one module address (byte index N <-> module 0xN), and bit 0 of
// that byte is the "installed" flag. Additional bits encode coding variant,
// subnet, etc. Treat the bytes you don't touch as black boxes: read, modify
// the single target byte, write the exact same buffer back.
//
// Module 0x75 = Telematics / Emergency Call Module (NAR: OCU). On a USA spec
// Tiguan R-Line Allspace the OCU is typically not populated, yet the Gateway
// ships with bit 0 of byte 0x75 set, which produces the "Emergency Call
// Function Impaired" warning. Clearing that bit removes the warning.
// ---------------------------------------------------------------------------
struct InstallationList {
    std::vector<uint8_t> raw;   // exact bytes as read from the ECU

    // Is module `addr` currently marked as installed?
    bool isInstalled(uint8_t addr) const;

    // Clear the "installed" bit for module `addr`. Returns false if `addr`
    // is out of range for the list currently held.
    bool setInstalled(uint8_t addr, bool installed);
};

// Pretty-print an Installation List as `addr: xx  [INSTALLED]` lines.
void printInstallationList(const InstallationList& list, std::ostream& os);

// ---------------------------------------------------------------------------
// High-level task helpers.
// ---------------------------------------------------------------------------

// Read VIN + part number + SW version for identification/safety check.
struct GatewayIdentity {
    std::string vin;
    std::string part_number;
    std::string sw_version;
    std::string serial_number;
};
bool readGatewayIdentity(uds::UdsClient& client, GatewayIdentity& out);

// Read Installation List (DID 0x0608).
bool readInstallationList(uds::UdsClient& client, InstallationList& out);

// Write a modified Installation List back (DID 0x0608).
// `dry_run` = true only prints what it *would* send.
bool writeInstallationList(uds::UdsClient& client,
                           const InstallationList& list,
                           bool dry_run);

// Fix "Emergency Call Function Impaired":
//   1. Read DID 0x0608
//   2. Clear bit 0 of byte 0x75
//   3. Write DID 0x0608
// Assumes the caller has already entered the extended session and unlocked
// Security Access.
bool fixEmergencyCall(uds::UdsClient& client, bool dry_run);

// Disable the Start-Stop system at Gateway level by toggling the appropriate
// long-coding byte at DID 0x0600. Byte index / bit index / polarity vary by
// vehicle -- the defaults here match the most common MQB documentation, but
// you MUST verify against your own coding dump. Prints a diff before writing.
bool disableStartStop(uds::UdsClient& client, bool dry_run,
                      size_t coding_byte_index = 14,   // vehicle-specific
                      uint8_t bit_mask          = 0x20, // vehicle-specific
                      bool    clear_bit_to_disable = true);

} // namespace vw
