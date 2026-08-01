#include "crc16.hpp"

namespace uart {

uint16_t Crc16Ccitt::update(uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
        if (crc & 0x8000) {
            crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t Crc16Ccitt::compute(const uint8_t* data, size_t len) {
    uint16_t crc = kInit;
    for (size_t i = 0; i < len; ++i) {
        crc = update(crc, data[i]);
    }
    return crc; // xor-out is 0x0000, so nothing further to do
}

} // namespace uart
