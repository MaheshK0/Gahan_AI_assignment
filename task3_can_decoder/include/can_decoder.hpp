#pragma once
#include "dbc.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <variant>

namespace can {

using RawFrame = std::array<uint8_t, 8>;

// Extract `length` bits starting at `start_bit` from an 8-byte CAN frame,
// per the byte_order convention, and return the raw (unscaled, unsigned
// bit pattern) integer value. Signedness/scale/offset are applied by the
// caller (decodeSignal below) -- this function only does bit-shuffling.
uint64_t extractRawBits(const RawFrame& data, unsigned start_bit, unsigned length, ByteOrder order);

// Applies signedness, scale and offset to a raw bit pattern, returning a
// physical-unit double.
double decodeSignal(const RawFrame& data, const SignalDef& sig, ByteOrder order);

// XOR of all bytes in the frame except the one occupied by `checksum_sig`.
// Every message in this DBC uses a single, byte-aligned checksum byte, and
// vehicle.dbc's CM_ comments state the checksum is "XOR of the other 7
// bytes in the frame" -- this was independently re-verified against every
// frame in frames.log (see DESIGN.md) before being adopted here.
uint8_t computeXorChecksum(const RawFrame& data, const SignalDef& checksum_sig);

} // namespace can
