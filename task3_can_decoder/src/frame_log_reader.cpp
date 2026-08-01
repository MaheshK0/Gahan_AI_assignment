#include "frame_log_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace can {

std::vector<LogFrame> FrameLogReader::readAll(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Could not open frames.log: " + path);

    std::vector<LogFrame> frames;
    std::string line;
    std::vector<std::string> record;

    auto flushRecord = [&]() {
        if (record.empty()) return;
        if (record.size() != 3) {
            throw std::runtime_error("frames.log: malformed record (expected 3 lines, got " +
                                      std::to_string(record.size()) + ")");
        }
        LogFrame frame;
        frame.timestamp_ms = std::stol(record[0]);
        frame.can_id = static_cast<uint32_t>(std::stoul(record[1], nullptr, 16));

        std::istringstream ss(record[2]);
        std::string byteStr;
        size_t i = 0;
        while (ss >> byteStr && i < frame.data.size()) {
            frame.data[i++] = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
        }
        frames.push_back(frame);
        record.clear();
    };

    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) {
            flushRecord();
            continue;
        }
        record.push_back(line);
    }
    flushRecord();

    return frames;
}

} // namespace can
