#include "uart_parser.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

std::vector<uint8_t> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::runtime_error("Could not open file: " + path);
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    const std::string s = ss.str();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void printPacket(const uart::Packet& pkt) {
    std::printf("PACKET  type=0x%02X  len=%3u  payload=", pkt.type, pkt.length);
    for (uint8_t i = 0; i < pkt.length; ++i) {
        std::printf("%02X ", pkt.payload[i]);
    }
    std::printf("\n");
}

} // namespace

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1]
                                         : "task1_uart_parser/data/uart_stream.bin";

    std::vector<uint8_t> data;
    try {
        data = readFile(path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Usage: " << argv[0] << " [path/to/uart_stream.bin]\n";
        return 1;
    }

    uart::UartParser parser(printPacket);

    // Requirement: process one byte at a time via feed(byte). We do this
    // explicitly in the driver loop (rather than using the chunked
    // feed(data,len) convenience overload) to make that requirement visibly
    // satisfied; the chunked overload is exercised in the unit tests
    // instead, covering the bonus "arbitrary chunk size" requirement.
    for (uint8_t byte : data) {
        parser.feed(byte);
    }
    parser.finish();

    const auto& stats = parser.stats();
    std::printf("\n--- Parser statistics ---\n");
    std::printf("valid_packets    : %llu\n", static_cast<unsigned long long>(stats.valid_packets));
    std::printf("crc_failures     : %llu\n", static_cast<unsigned long long>(stats.crc_failures));
    std::printf("sync_losses      : %llu\n", static_cast<unsigned long long>(stats.sync_losses));
    std::printf("discarded_bytes  : %llu\n", static_cast<unsigned long long>(stats.discarded_bytes));
    std::printf("partial_packets  : %llu\n", static_cast<unsigned long long>(stats.partial_packets));

    return 0;
}
