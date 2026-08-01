#include "dbc.hpp"
#include "json_lite.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace can {

namespace {

uint32_t parseCanIdKey(const std::string& key) {
    // Keys look like "0x180".
    return static_cast<uint32_t>(std::stoul(key, nullptr, 16));
}

SignalDef parseSignal(const std::string& name, const JsonValue& v) {
    SignalDef sig;
    sig.name = name;
    sig.start_bit = static_cast<unsigned>(v.at("start_bit").asNumber());
    sig.length = static_cast<unsigned>(v.at("length").asNumber());
    sig.scale = v.at("scale").asNumber();
    sig.offset = v.at("offset").asNumber();
    sig.is_signed = v.at("signed").asBool();
    if (v.has("unit")) sig.unit = v.at("unit").asString();
    if (v.has("enum")) {
        for (const auto& [k, val] : v.at("enum").members()) {
            sig.enum_values[std::stoi(k)] = val.asString();
        }
    }
    return sig;
}

} // namespace

DbcDatabase DbcDatabase::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Could not open dbc.json: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();

    JsonValue root = JsonValue::parse(ss.str());
    if (!root.isObject()) throw std::runtime_error("dbc.json: expected top-level object");

    DbcDatabase db;
    for (const auto& [idKey, msgVal] : root.members()) {
        MessageDef msg;
        msg.can_id = parseCanIdKey(idKey);
        msg.name = msgVal.at("name").asString();
        msg.period_ms = static_cast<unsigned>(msgVal.at("period_ms").asNumber());
        std::string bo = msgVal.at("byte_order").asString();
        msg.byte_order = (bo == "motorola") ? ByteOrder::Motorola : ByteOrder::Intel;

        const JsonValue& sigs = msgVal.at("signals");
        for (const auto& [sigName, sigVal] : sigs.members()) {
            msg.signals.push_back(parseSignal(sigName, sigVal));
        }
        db.messages_[msg.can_id] = std::move(msg);
    }
    return db;
}

const MessageDef* DbcDatabase::find(uint32_t can_id) const {
    auto it = messages_.find(can_id);
    return it == messages_.end() ? nullptr : &it->second;
}

} // namespace can
