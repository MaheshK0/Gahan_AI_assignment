#include "dbc_database.hpp"
#include "json_value.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace can {

namespace {

uint32_t parseHexId(const std::string& s) {
    return static_cast<uint32_t>(std::stoul(s, nullptr, 16));
}

} // namespace

DbcDatabase DbcDatabase::loadFromJsonFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Could not open DBC JSON file: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();

    JsonValue root = JsonValue::parse(ss.str());
    if (!root.isObject()) throw std::runtime_error("dbc.json: top level must be an object");

    DbcDatabase db;
    for (const auto& [idKey, msgVal] : root.members()) {
        MessageDef msg;
        msg.id = parseHexId(idKey);
        msg.name = msgVal.at("name").asString();
        msg.period_ms = static_cast<int>(msgVal.at("period_ms").asLong());
        const std::string bo = msgVal.at("byte_order").asString();
        msg.byte_order = (bo == "motorola") ? ByteOrder::Motorola : ByteOrder::Intel;

        const JsonValue& sigsVal = msgVal.at("signals");
        for (const auto& [sigName, sigVal] : sigsVal.members()) {
            SignalDef sig;
            sig.name = sigName;
            sig.start_bit = static_cast<int>(sigVal.at("start_bit").asLong());
            sig.length = static_cast<int>(sigVal.at("length").asLong());
            sig.scale = sigVal.has("scale") ? sigVal.at("scale").asDouble() : 1.0;
            sig.offset = sigVal.has("offset") ? sigVal.at("offset").asDouble() : 0.0;
            sig.is_signed = sigVal.has("signed") && sigVal.at("signed").asBool();
            sig.unit = sigVal.has("unit") ? sigVal.at("unit").asString() : "";
            if (sigVal.has("enum")) {
                for (const auto& [k, v] : sigVal.at("enum").members()) {
                    sig.enum_values[std::stoi(k)] = v.asString();
                }
            }
            msg.signals.push_back(std::move(sig));
        }

        db.messages_[msg.id] = std::move(msg);
    }
    return db;
}

const MessageDef* DbcDatabase::find(uint32_t id) const {
    auto it = messages_.find(id);
    return it == messages_.end() ? nullptr : &it->second;
}

} // namespace can
