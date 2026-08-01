#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace can {

enum class ByteOrder { Intel, Motorola };

struct SignalDef {
    std::string name;
    int start_bit = 0;
    int length = 0;
    double scale = 1.0;
    double offset = 0.0;
    bool is_signed = false;
    std::string unit;
    std::map<int, std::string> enum_values; // optional value table, e.g. gear
};

struct MessageDef {
    uint32_t id = 0;
    std::string name;
    int period_ms = 0;
    ByteOrder byte_order = ByteOrder::Intel;
    std::vector<SignalDef> signals;

    const SignalDef* findSignal(const std::string& name) const {
        for (const auto& s : signals) {
            if (s.name == name) return &s;
        }
        return nullptr;
    }
};

// In-memory signal database, loaded from dbc.json. (The equivalent
// vehicle.dbc text file describes the identical set of messages; dbc.json
// is used here because it's unambiguous to parse without having to
// reimplement the DBC format's big-endian bit-numbering quirks from
// scratch -- see DESIGN.md for the reasoning and for how the Motorola
// byte-order convention used here was empirically verified against the
// sample frames.)
class DbcDatabase {
public:
    static DbcDatabase loadFromJsonFile(const std::string& path);

    const MessageDef* find(uint32_t id) const;
    const std::map<uint32_t, MessageDef>& messages() const { return messages_; }

private:
    std::map<uint32_t, MessageDef> messages_;
};

} // namespace can
