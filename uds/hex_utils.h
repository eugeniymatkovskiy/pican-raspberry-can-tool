#pragma once

#include <cstdint>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace hexu {

inline std::string bytesToHex(const uint8_t* data, size_t n, char sep = ' ') {
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < n; ++i) {
        if (i && sep) os << sep;
        os << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    return os.str();
}

inline std::string bytesToHex(const std::vector<uint8_t>& v, char sep = ' ') {
    return bytesToHex(v.data(), v.size(), sep);
}

inline std::string idToHex(uint32_t id, bool extended) {
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0')
       << std::setw(extended ? 8 : 3) << id;
    return os.str();
}

// Parse "DEADBEEF" or "DE AD BE EF" into a byte vector. Returns false on malformed input.
inline bool parseHexString(const std::string& in, std::vector<uint8_t>& out) {
    out.clear();
    std::string compact;
    compact.reserve(in.size());
    for (char c : in) {
        if (c == ' ' || c == ':' || c == '-' || c == '_') continue;
        compact.push_back(c);
    }
    if (compact.size() % 2 != 0) return false;
    for (size_t i = 0; i < compact.size(); i += 2) {
        auto hexVal = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
            if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
            return -1;
        };
        int hi = hexVal(compact[i]);
        int lo = hexVal(compact[i + 1]);
        if (hi < 0 || lo < 0) return false;
        out.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return true;
}

} // namespace hexu
