#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace can {

struct LogFrame {
    long timestamp_ms = 0;
    uint32_t can_id = 0;
    std::array<uint8_t, 8> data{};
};

// Parses frames.log: blank-line-separated 3-line records of
//   <timestamp_ms>
//   0x<CAN ID hex>
//   <8 space-separated hex data bytes>
class FrameLogReader {
public:
    static std::vector<LogFrame> readAll(const std::string& path);
};

} // namespace can
