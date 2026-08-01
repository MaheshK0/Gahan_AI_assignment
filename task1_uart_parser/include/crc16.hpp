#pragma once
#include <cstdint>
#include <cstddef>

// CRC-16/CCITT-FALSE
// Poly: 0x1021, Init: 0xFFFF, RefIn: false, RefOut: false, XorOut: 0x0000
namespace uart {

class Crc16Ccitt {
public:
    // Feed one byte at a time into a running CRC (starts at kInit).
    static uint16_t update(uint16_t crc, uint8_t byte);

    // Convenience: compute CRC over a contiguous buffer, starting from kInit.
    static uint16_t compute(const uint8_t* data, size_t len);

    static constexpr uint16_t kInit = 0xFFFF;
};

} // namespace uart
