#include "signal_codec.hpp"

namespace can {

uint64_t SignalCodec::extractRaw(const FramePayload& data, const SignalDef& sig, ByteOrder order) {
    uint64_t value = 0;

    if (order == ByteOrder::Intel) {
        for (int i = 0; i < sig.length; ++i) {
            const int bitpos = sig.start_bit + i;
            const int byte_idx = bitpos / 8;
            const int bit_in_byte = bitpos % 8; // 0 = LSB of that byte
            if (byte_idx < 0 || byte_idx >= static_cast<int>(data.size())) continue;
            const uint64_t bit = (data[byte_idx] >> bit_in_byte) & 0x1;
            value |= (bit << i); // accumulate LSB-first
        }
    } else { // Motorola: big-endian bit stream, MSB-first accumulation
        for (int i = 0; i < sig.length; ++i) {
            const int bitpos = sig.start_bit + i;
            const int byte_idx = bitpos / 8;
            const int bit_in_byte_from_msb = bitpos % 8; // 0 = MSB of that byte
            if (byte_idx < 0 || byte_idx >= static_cast<int>(data.size())) continue;
            const int bit_in_byte = 7 - bit_in_byte_from_msb;
            const uint64_t bit = (data[byte_idx] >> bit_in_byte) & 0x1;
            value = (value << 1) | bit; // accumulate MSB-first
        }
    }
    return value;
}

int64_t SignalCodec::extractSigned(const FramePayload& data, const SignalDef& sig, ByteOrder order) {
    uint64_t raw = extractRaw(data, sig, order);
    if (!sig.is_signed || sig.length == 0 || sig.length >= 64) {
        return static_cast<int64_t>(raw);
    }
    const uint64_t sign_bit = uint64_t(1) << (sig.length - 1);
    if (raw & sign_bit) {
        // Sign-extend: fill all bits above `length` with 1s.
        const uint64_t mask = ~uint64_t(0) << sig.length;
        raw |= mask;
    }
    return static_cast<int64_t>(raw);
}

double SignalCodec::decodePhysical(const FramePayload& data, const SignalDef& sig, ByteOrder order) {
    const double raw = sig.is_signed
        ? static_cast<double>(extractSigned(data, sig, order))
        : static_cast<double>(extractRaw(data, sig, order));
    return raw * sig.scale + sig.offset;
}

} // namespace can
