#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace can {

enum class ByteOrder { Intel, Motorola };

struct SignalDef {
    std::string name;
    unsigned start_bit = 0;
    unsigned length = 0;
    double scale = 1.0;
    double offset = 0.0;
    bool is_signed = false;
    std::string unit;
    std::map<int, std::string> enum_values; // optional, e.g. gear -> "P"/"R"/...
};

struct MessageDef {
    uint32_t can_id = 0;
    std::string name;
    unsigned period_ms = 0;
    ByteOrder byte_order = ByteOrder::Intel;
    std::vector<SignalDef> signals;

    const SignalDef* findSignal(const std::string& name) const {
        for (const auto& s : signals) {
            if (s.name == name) return &s;
        }
        return nullptr;
    }
};

// Loads the message/signal database from dbc.json. Empirically verified
// (see DESIGN.md) against the sample data:
//   - Intel (little-endian) signals: start_bit is the bit position of the
//     signal's LSB, using the standard "byte*8 + bit" linear numbering;
//     bits are read from start_bit upward, least-significant-first.
//   - Motorola (big-endian) signals: start_bit is the bit position of the
//     signal's MSB, using the Vector/DBC big-endian numbering where
//     byte_index = start_bit / 8 and bit_in_byte = start_bit % 8 (7 = MSB
//     of that byte); bits are read from there towards the LSB of the
//     field, spilling into the next byte's bit 7 when a byte boundary is
//     crossed. (Note: this differs from a naive "bit-reversed" reading of
//     the DBC bit number -- see DESIGN.md for how this was verified.)
class DbcDatabase {
public:
    // dbc.json and vehicle.dbc describe the same messages, but their
    // start_bit fields use *different* bit-numbering conventions (verified
    // empirically against frames.log -- see DESIGN.md). This project uses
    // loadFromDbcText() as the actual source of truth for decoding, since
    // its convention (the standard Vector/DBC one) was independently
    // cross-checked against every frame in the sample log with 100%
    // agreement on decoded ranges, counter sequences and checksums.
    // loadFromJson() is kept as an alternate loader (same MessageDef
    // output shape) to show both are supported, but is not what the demo
    // uses by default.
    static DbcDatabase loadFromJson(const std::string& path);
    static DbcDatabase loadFromDbcText(const std::string& path);

    const std::map<uint32_t, MessageDef>& messages() const { return messages_; }
    const MessageDef* find(uint32_t can_id) const;

private:
    std::map<uint32_t, MessageDef> messages_;
};

} // namespace can
