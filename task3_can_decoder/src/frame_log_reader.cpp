#include "frame_log_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace can {

namespace {

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

} // namespace

std::vector<RawFrame> FrameLogReader::readAll(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Could not open frame log: " + path);

    std::vector<RawFrame> frames;
    std::string line;
    int fieldIdx = 0; // 0=timestamp, 1=id, 2=payload
    RawFrame current;

    auto flushIfComplete = [&](bool haveAllFields) {
        if (haveAllFields) {
            frames.push_back(current);
        }
        current = RawFrame{};
        fieldIdx = 0;
    };

    while (std::getline(f, line)) {
        std::string t = trim(line);
        if (t.empty()) {
            // Blank line: if we already had a complete record buffered,
            // this is just the separator after it (already flushed below);
            // if we're mid-record with too few fields, treat as a
            // malformed/incomplete record and skip it rather than crash.
            continue;
        }
        if (fieldIdx == 0) {
            current.timestamp_ms = std::stol(t);
            fieldIdx = 1;
        } else if (fieldIdx == 1) {
            current.can_id = static_cast<uint32_t>(std::stoul(t, nullptr, 16));
            fieldIdx = 2;
        } else if (fieldIdx == 2) {
            std::istringstream iss(t);
            std::string byteStr;
            int i = 0;
            current.data.fill(0);
            while (iss >> byteStr && i < 8) {
                current.data[i++] = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
            }
            current.dlc = i;
            flushIfComplete(true);
        }
    }
    // Any dangling partial record at EOF (fieldIdx != 0) is silently
    // dropped rather than causing a crash -- a malformed trailing record
    // isn't decodable and shouldn't take down the whole replay.
    return frames;
}

} // namespace can
