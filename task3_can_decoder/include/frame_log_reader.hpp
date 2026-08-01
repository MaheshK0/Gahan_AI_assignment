#pragma once
#include "signal_codec.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace can {

// One captured frame from frames.log.
struct RawFrame {
    long timestamp_ms = 0;
    uint32_t can_id = 0;
    FramePayload data{};
    int dlc = 0; // number of payload bytes actually present (<=8)
};

// Reads frames.log, which is formatted as repeated 3-line blocks separated
// by blank lines:
//   <timestamp_ms>
//   <hex CAN ID, e.g. 0x180>
//   <space-separated hex payload bytes, e.g. "9C 0C 00 90 00 00 00 00">
class FrameLogReader {
public:
    static std::vector<RawFrame> readAll(const std::string& path);
};

} // namespace can
