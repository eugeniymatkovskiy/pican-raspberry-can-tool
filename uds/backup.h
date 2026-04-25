#pragma once

// Backup file format for ELM-UDS. Purpose: before we ever write anything back
// to the Gateway, we capture a complete snapshot of everything we might touch
// so we can (a) roll back if something goes wrong, and (b) assert the car we
// are writing to is the same car we read from.
//
// Format (plain text, human-inspectable, forward/backward compatible):
//
//     # ELM-UDS backup v1
//     vin=WVGBV7AX7LK000001
//     part_number=5Q0907530D
//     sw_version=0271
//     serial=SN12345678
//     timestamp=2026-04-25T21:00:00Z
//     coding_did=0600
//     coding=01 02 03 0A 0B 0C ...
//     installation_list_did=0608
//     installation_list=00 00 01 00 ... 01 ...
//
// Why plain text? So you can open the backup in any editor, diff two of them,
// paste them into VCDS reports, and -- most importantly -- reconstruct the
// write manually by hand if this tool ever breaks.

#include "uds_client.h"
#include "vw_mqb.h"

#include <cstdint>
#include <string>
#include <vector>

namespace backup {

struct Data {
    // Identity fingerprint. We'll refuse to restore/write to a Gateway
    // whose VIN + part_number doesn't match this.
    std::string vin;
    std::string part_number;
    std::string sw_version;
    std::string serial;
    std::string timestamp;   // ISO-8601 UTC

    // Full coding (DID 0x0600) and installation list (DID 0x0608), exactly
    // as read from the ECU -- no mutation.
    std::vector<uint8_t> coding;
    std::vector<uint8_t> installation_list;

    bool empty() const { return coding.empty() && installation_list.empty(); }
};

// Read identity + DID 0x0600 + DID 0x0608 from the ECU and populate `out`.
// Caller must already be in the extended session. Prints progress to stdout.
bool capture(uds::UdsClient& client, Data& out);

// Write backup to disk. Returns false on I/O error.
bool save(const std::string& path, const Data& data);

// Load backup from disk. Returns false on I/O error / bad format.
bool load(const std::string& path, Data& out);

// Verify that `backup` matches the identity currently reported by `client`.
// Returns true only if VIN and part_number match. SW version mismatch is a
// *warning* (software updates happen), not a hard failure.
bool verifyIdentity(uds::UdsClient& client, const Data& backup,
                    bool& sw_version_changed);

// Write back the saved buffers (coding then installation list), with an
// identity check first and a readback-verify after each write.
// If `dry_run` is true, only prints the planned writes.
bool restore(uds::UdsClient& client, const Data& backup, bool dry_run);

// Human-readable dump to stdout.
void print(const Data& data);

} // namespace backup
