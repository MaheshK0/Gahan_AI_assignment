#pragma once
#include "dbc_database.hpp"

#include <array>
#include <cstdint>

namespace can {

using FramePayload = std::array<uint8_t, 8>;

// Extracts and decodes CAN signals from a raw 8-byte payload, per the bit
// layout / byte-order / scaling described in the signal's DBC definition.
//
// Bit-numbering conventions (both verified empirically against the
// provided frames.log -- see DESIGN.md):
//
//   Intel / little-endian (byte_order == Intel):
//     start_bit is the index of the signal's LEAST-significant bit,
//     counting bit 0 as the LSB of payload byte 0 and increasing upward
//     through the byte array (bit 8 = LSB of byte 1, etc). Bits are
//     accumulated LSB-first as start_bit increases.
//
//   Motorola / big-endian (byte_order == Motorola):
//     start_bit is the index of the signal's MOST-significant bit,
//     counting bit 0 as the MSB of payload byte 0 and increasing
//     sequentially through the byte array in normal reading order (so
//     bits 0..7 span byte 0 MSB-to-LSB, bits 8..15 span byte 1
//     MSB-to-LSB, etc). Bits are accumulated MSB-first as start_bit
//     increases -- i.e. the payload is treated as one big-endian bit
//     stream and `length` bits are read starting at `start_bit`.
//
// Both signed and unsigned signals are supported; signed signals are
// sign-extended from `length` bits.
class SignalCodec {
public:
    // Raw (unscaled) integer value of the signal.
    static uint64_t extractRaw(const FramePayload& data, const SignalDef& sig, ByteOrder order);

    // Signed/unsigned interpretation of the raw bits.
    static int64_t extractSigned(const FramePayload& data, const SignalDef& sig, ByteOrder order);

    // Fully decoded physical value: raw * scale + offset (sign-extended
    // first if sig.is_signed).
    static double decodePhysical(const FramePayload& data, const SignalDef& sig, ByteOrder order);
};

} // namespace can
