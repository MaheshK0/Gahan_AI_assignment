#include "can_decoder.hpp"

namespace can {

namespace {

uint64_t extractIntel(const RawFrame& data, unsigned start_bit, unsigned length) {
    uint64_t val = 0;
    for (unsigned i = 0; i < length; ++i) {
        unsigned bitpos = start_bit + i;
        unsigned byte_idx = bitpos / 8;
        unsigned bit_idx = bitpos % 8;
        if (byte_idx >= data.size()) break; // out-of-range guard
        uint64_t bit = (data[byte_idx] >> bit_idx) & 0x1u;
        val |= (bit << i);
    }
    return val;
}

uint64_t extractMotorola(const RawFrame& data, unsigned start_bit, unsigned length) {
    // Vector/DBC big-endian bit numbering: start_bit is the MSB of the
    // signal. byte_idx = start_bit / 8, bit_in_byte = start_bit % 8 (7 =
    // MSB of that byte). Bits are consumed from there towards the LSB
    // (bit_in_byte decreasing); crossing a byte boundary wraps to bit 7 of
    // the next byte. Verified against the SteeringData signals in
    // frames.log -- see DESIGN.md.
    unsigned byte_idx = start_bit / 8;
    int bit_in_byte = static_cast<int>(start_bit % 8);
    uint64_t val = 0;
    for (unsigned i = 0; i < length; ++i) {
        if (byte_idx >= data.size()) break; // out-of-range guard
        uint64_t bit = (data[byte_idx] >> bit_in_byte) & 0x1u;
        val = (val << 1) | bit;
        --bit_in_byte;
        if (bit_in_byte < 0) {
            bit_in_byte = 7;
            ++byte_idx;
        }
    }
    return val;
}

int64_t toSigned(uint64_t raw, unsigned length) {
    if (length == 0 || length >= 64) return static_cast<int64_t>(raw);
    const uint64_t signBit = uint64_t(1) << (length - 1);
    if (raw & signBit) {
        return static_cast<int64_t>(raw) - static_cast<int64_t>(uint64_t(1) << length);
    }
    return static_cast<int64_t>(raw);
}

} // namespace

uint64_t extractRawBits(const RawFrame& data, unsigned start_bit, unsigned length, ByteOrder order) {
    return (order == ByteOrder::Intel) ? extractIntel(data, start_bit, length)
                                        : extractMotorola(data, start_bit, length);
}

double decodeSignal(const RawFrame& data, const SignalDef& sig, ByteOrder order) {
    uint64_t raw = extractRawBits(data, sig.start_bit, sig.length, order);
    double physical;
    if (sig.is_signed) {
        physical = static_cast<double>(toSigned(raw, sig.length));
    } else {
        physical = static_cast<double>(raw);
    }
    return physical * sig.scale + sig.offset;
}

uint8_t computeXorChecksum(const RawFrame& data, const SignalDef& checksum_sig) {
    unsigned checksum_byte = checksum_sig.start_bit / 8;
    uint8_t x = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i == checksum_byte) continue;
        x ^= data[i];
    }
    return x;
}

} // namespace can
