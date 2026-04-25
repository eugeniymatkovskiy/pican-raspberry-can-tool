#include "iso_tp.h"
#include "hex_utils.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace isotp {

const char* statusToString(Status s) {
    switch (s) {
        case Status::Ok:             return "Ok";
        case Status::TimeoutRx:      return "TimeoutRx";
        case Status::TimeoutFc:      return "TimeoutFc";
        case Status::SocketError:    return "SocketError";
        case Status::UnexpectedPci:  return "UnexpectedPci";
        case Status::SequenceError:  return "SequenceError";
        case Status::OverflowFc:     return "OverflowFc";
        case Status::TooLong:        return "TooLong";
        case Status::BadFrame:       return "BadFrame";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// STmin ("minimum separation time") is a byte in the Flow Control frame that
// tells the *sender* how fast it is allowed to fire Consecutive Frames.
//
// Think of it as the receiver's rate-limiter knob. Values:
//   0x00..0x7F  -> 0..127 milliseconds between CFs
//   0xF1..0xF9  -> 100..900 microseconds between CFs (sub-millisecond)
//   anything else -> weird/reserved; we pick a safe upper bound (127ms)
// so we never spam an ECU faster than it asked for.
// ---------------------------------------------------------------------------
static std::chrono::microseconds decodeStMin(uint8_t st) {
    if (st <= 0x7F)
        return std::chrono::milliseconds(st);
    if (st >= 0xF1 && st <= 0xF9)
        return std::chrono::microseconds((st - 0xF0) * 100);
    return std::chrono::milliseconds(127);
}

IsoTp::IsoTp(int socket_fd, const Config& cfg) : fd_(socket_fd), cfg_(cfg) {}

void IsoTp::logFrame(const char* dir, uint32_t id, const uint8_t data[8]) {
    if (!cfg_.verbose) return;
    std::cout << "[ISO-TP " << dir << "] ID="
              << hexu::idToHex(id, cfg_.extended_id)
              << "  [" << hexu::bytesToHex(data, 8) << "]"
              << std::endl;
}

// Write exactly one 8-byte CAN frame to the bus.
//
// A `struct can_frame` is the Linux-level representation of *one* CAN frame:
//   can_id  - 11-bit or 29-bit identifier. For 29-bit we OR in CAN_EFF_FLAG
//             (0x80000000) to tell the kernel "this is extended addressing".
//   can_dlc - how many data bytes are valid (1..8). VW expects all 8 bytes
//             populated (padded), so we always send 8.
//   data[8] - the actual payload.
//
// The kernel accepts the whole struct via a single write() syscall. This is
// the lowest-level building block -- everything else in ISO-TP is just
// "which bytes do I put in those 8 slots".
bool IsoTp::writeCanFrame(const uint8_t data[8]) {
    struct can_frame f{};
    f.can_id  = cfg_.tx_id | (cfg_.extended_id ? CAN_EFF_FLAG : 0);
    f.can_dlc = 8;
    std::memcpy(f.data, data, 8);
    ssize_t n = ::write(fd_, &f, sizeof(f));
    if (n != static_cast<ssize_t>(sizeof(f))) {
        std::cerr << "[ISO-TP] write() failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    logFrame("TX", cfg_.tx_id, data);
    return true;
}

int IsoTp::readCanFrame(uint8_t out[8], int timeout_ms) {
    // select() gives us a clean timeout without relying on SO_RCVTIMEO state.
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rv = ::select(fd_ + 1, &rset, nullptr, nullptr, &tv);
    if (rv < 0) {
        std::cerr << "[ISO-TP] select() failed: " << std::strerror(errno) << std::endl;
        return -1;
    }
    if (rv == 0) return 0; // timeout

    struct can_frame f{};
    ssize_t n = ::read(fd_, &f, sizeof(f));
    if (n != static_cast<ssize_t>(sizeof(f))) {
        std::cerr << "[ISO-TP] read() short/failed: " << std::strerror(errno) << std::endl;
        return -1;
    }

    // Always return 8 bytes, zero-pad if the frame came in short (shouldn't
    // normally happen on a well-formed bus, but don't fall apart if it does).
    std::memset(out, 0, 8);
    std::memcpy(out, f.data, f.can_dlc);

    uint32_t rid = f.can_id & (cfg_.extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
    logFrame("RX", rid, out);
    return 1;
}

// ---------------------------------------------------------------------------
// send(): take a UDS message of any length and put it on the bus.
//
// High-level decision tree:
//   payload.size() <= 7   -> one Single Frame, done.
//   payload.size()  > 7   -> one First Frame, wait for Flow Control,
//                            then a stream of Consecutive Frames.
//
// See GUIDE.md sections 4.1 and 4.2 for the bytes-on-the-wire picture that
// this code is building.
// ---------------------------------------------------------------------------
Status IsoTp::send(const std::vector<uint8_t>& payload) {
    if (payload.empty()) return Status::BadFrame;
    if (payload.size() > 4095) return Status::TooLong;  // 12-bit FF length cap

    uint8_t frame[8];

    // =========================================================
    // Case A: Single Frame. Everything fits in one CAN frame.
    //
    //   [ 0L ][ payload bytes (1..7) ][ padding up to byte 7 ]
    //      ^
    //      L = length of the payload, high nibble is 0 (= "SF").
    //
    // Example: UDS "10 03" (DiagSession = Extended) becomes:
    //   [ 02 ][ 10 ][ 03 ][ AA ][ AA ][ AA ][ AA ][ AA ]
    // =========================================================
    if (payload.size() <= 7) {
        std::memset(frame, cfg_.padding, 8);
        frame[0] = static_cast<uint8_t>(payload.size() & 0x0F);   // N_PCI = 0x0L
        std::memcpy(&frame[1], payload.data(), payload.size());
        if (!writeCanFrame(frame)) return Status::SocketError;
        return Status::Ok;
    }

    // =========================================================
    // Case B: First Frame. Header is 2 bytes, leaving 6 bytes
    // of actual payload in this very first frame.
    //
    //   byte[0] = 0x1X    <- '1' in high nibble tells the peer
    //                        "I'm starting a multi-frame message".
    //                        X is the top 4 bits of the 12-bit total length.
    //   byte[1] =  YY     <- bottom 8 bits of the total length.
    //
    // So for a 131-byte total: 131 = 0x083 -> byte[0]=0x10, byte[1]=0x83.
    //
    //   payload.size() >> 8  == top 8 bits of the length. We only keep the
    //                           bottom 4 (`& 0x0F`) because the high nibble
    //                           of byte[0] must be the FF marker '1'.
    //   `0x10 | (...)`        merges "FF marker" with "length high nibble".
    //   payload.size() & 0xFF  == bottom 8 bits of the length -> byte[1].
    // =========================================================
    {
        std::memset(frame, cfg_.padding, 8);
        frame[0] = static_cast<uint8_t>(0x10 | ((payload.size() >> 8) & 0x0F));
        frame[1] = static_cast<uint8_t>(payload.size() & 0xFF);
        std::memcpy(&frame[2], payload.data(), 6);   // bytes 2..7 = first 6 of payload
        if (!writeCanFrame(frame)) return Status::SocketError;
    }

    // We've now sent 6 bytes. The remainder goes out as Consecutive Frames.
    size_t sent = 6;

    // Sequence counter for CFs. ISO-TP rule: *first* CF is sequence 1 (NOT 0;
    // 0 is reserved because 0x20 would collide with... actually 0x20 is a
    // valid CF with SN=0, but the FIRST CF after the FF must be 1). From
    // there we count up by one, and when we hit 0xF (15) we wrap to 0x0, not
    // back to 1. This wrap is the single most-tested-for bug in ISO-TP
    // implementations -- see tests/isotp_loopback.cpp.
    uint8_t seq = 1;

    // Flow Control negotiated values. `remaining_in_block == 0` is our flag
    // that we are "waiting for an FC" (either haven't received the first FC
    // after the FF, or we've exhausted the block size the ECU granted us).
    uint8_t remaining_in_block = 0;
    auto    min_gap = std::chrono::microseconds(0);

    while (sent < payload.size()) {
        // --- Phase 1: make sure we have permission to send more CFs. ---
        if (remaining_in_block == 0) {
            uint8_t fc[8];

            // An FC can come as "Wait" (FS=1) several times in a row while
            // the ECU's internal buffer is busy; we simply read again until
            // we get CTS (FS=0) or Overflow (FS=2).
            for (;;) {
                int r = readCanFrame(fc, cfg_.timeout_ms);
                if (r == 0)  return Status::TimeoutFc;     // ECU never replied
                if (r < 0)   return Status::SocketError;

                // Sanity: first byte's high nibble MUST be 3 (Flow Control).
                if ((fc[0] & 0xF0) != 0x30) return Status::UnexpectedPci;

                uint8_t flag = fc[0] & 0x0F;   // 0=CTS, 1=Wait, 2=Overflow
                if (flag == 0x02) return Status::OverflowFc;
                if (flag == 0x01) continue;    // Wait -> re-read, try again
                if (flag != 0x00) return Status::UnexpectedPci;

                // CTS. Extract the two knobs the ECU just handed us:
                uint8_t bs    = fc[1];   // block size: # of CFs before next FC
                uint8_t stmin = fc[2];   // min gap between CFs

                // `bs == 0` means "send everything, I won't ask for another
                // FC". We represent that internally as 0xFF so the decrement
                // below never trips the "out of block" condition.
                remaining_in_block = (bs == 0) ? 0xFF : bs;
                min_gap = decodeStMin(stmin);
                break;
            }
        }

        // --- Phase 2: build one CF and put it on the bus. ---
        //
        //   byte[0] = 0x2S     where S = current 4-bit sequence counter.
        //   bytes 1..7 = next up to 7 bytes of the payload.
        std::memset(frame, cfg_.padding, 8);
        frame[0] = static_cast<uint8_t>(0x20 | (seq & 0x0F));
        size_t chunk = std::min<size_t>(7, payload.size() - sent);
        std::memcpy(&frame[1], payload.data() + sent, chunk);
        if (!writeCanFrame(frame)) return Status::SocketError;

        sent += chunk;
        seq  = (seq + 1) & 0x0F;  // 0..F wrap; this is the important mask
        if (remaining_in_block != 0xFF) --remaining_in_block;

        // Respect the ECU's rate-limit between frames (STmin).
        if (sent < payload.size() && min_gap.count() > 0) {
            std::this_thread::sleep_for(min_gap);
        }
    }

    return Status::Ok;
}

// ---------------------------------------------------------------------------
// recv(): read one whole UDS message from the bus, no matter how many CAN
// frames it takes to get there.
//
// We dispatch on the high nibble of the first byte we see:
//   0x0X  -> Single Frame; the entire message arrived in one shot.
//   0x1X  -> First Frame; we must reply with a Flow Control and read
//             Consecutive Frames until we've collected `total` bytes.
//   0x2X / 0x3X -> these are *middle/control* frames that should never be
//             the first thing we see; treat as protocol error.
// ---------------------------------------------------------------------------
Status IsoTp::recv(std::vector<uint8_t>& payload) {
    payload.clear();

    uint8_t frame[8];
    int r = readCanFrame(frame, cfg_.timeout_ms);
    if (r == 0)  return Status::TimeoutRx;
    if (r < 0)   return Status::SocketError;

    // `pci_hi` = the high nibble. We isolate it with `& 0xF0` which keeps
    // bits 4..7 and zeroes bits 0..3. (Remember: each hex digit = 4 bits.)
    uint8_t pci_hi = frame[0] & 0xF0;

    // ---- Single Frame (high nibble == 0) ----
    //   byte[0] = 0x0L  where L is the length (1..7).
    //   bytes 1..L are the payload; the rest is padding we ignore.
    if (pci_hi == 0x00) {
        uint8_t len = frame[0] & 0x0F;                 // low nibble
        if (len == 0 || len > 7) return Status::BadFrame;
        payload.assign(frame + 1, frame + 1 + len);
        return Status::Ok;
    }

    // ---- First Frame (high nibble == 1) ----
    //   byte[0] = 0x1X    X = top 4 bits of the 12-bit total length.
    //   byte[1] =  YY     bottom 8 bits of the total length.
    //   bytes 2..7 = first 6 bytes of the payload (6, not 7, because of the
    //                 2-byte header eating into the 8-byte frame).
    //
    // To reconstruct the length we shift the X nibble left 8 and OR in YY:
    //
    //     total = (frame[0] & 0x0F) << 8 | frame[1]
    //             ^^^^^^^^^^^^^^^^^   ^^    ^^^^^^^^
    //             keep only low nib   shift      or with low byte
    //
    // Example: FF "10 83 62 06 08 00 01 00" -> total = 0x083 = 131.
    if (pci_hi == 0x10) {
        uint16_t total = (static_cast<uint16_t>(frame[0] & 0x0F) << 8) | frame[1];
        if (total < 8 || total > 4095) return Status::BadFrame;

        payload.reserve(total);
        payload.insert(payload.end(), frame + 2, frame + 8);    // first 6 bytes

        // Reply with our Flow Control. For most use cases we advertise:
        //   CTS (FS=0), BS=0 (no further FCs needed), STmin=0 (fire away).
        // This is equivalent to saying "send everything, as fast as you like".
        uint8_t fc[8];
        std::memset(fc, cfg_.padding, 8);
        fc[0] = 0x30;                       // '3' = FC, '0' = Continue
        fc[1] = cfg_.rx_block_size;         // BS
        fc[2] = cfg_.rx_st_min;              // STmin
        if (!writeCanFrame(fc)) return Status::SocketError;

        // Now read Consecutive Frames until we've assembled `total` bytes.
        // Each CF has: byte[0] = 0x2S  where S is the expected sequence
        // counter starting at 1 and wrapping 0..F.
        uint8_t expected_seq = 1;
        while (payload.size() < total) {
            int rr = readCanFrame(frame, cfg_.timeout_ms);
            if (rr == 0) return Status::TimeoutRx;
            if (rr < 0)  return Status::SocketError;

            if ((frame[0] & 0xF0) != 0x20) return Status::UnexpectedPci;
            uint8_t seq = frame[0] & 0x0F;
            if (seq != expected_seq) return Status::SequenceError;

            // The last CF may have fewer than 7 valid bytes; we only copy
            // what's actually needed to reach `total`. The rest is padding.
            size_t need = total - payload.size();
            size_t take = std::min<size_t>(7, need);
            payload.insert(payload.end(), frame + 1, frame + 1 + take);
            expected_seq = (expected_seq + 1) & 0x0F;  // wrap F->0
        }
        return Status::Ok;
    }

    // If we got a CF (0x2X) or FC (0x3X) here, someone else is mid-conversation
    // on this CAN ID, or the ECU is in a weird state. Either way, not our
    // problem to recover from -- bail with a protocol error.
    return Status::UnexpectedPci;
}

} // namespace isotp
