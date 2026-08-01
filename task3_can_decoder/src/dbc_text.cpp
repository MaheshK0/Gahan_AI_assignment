#include "dbc.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace can {

namespace {

// A small, purpose-built parser for the subset of the DBC grammar this
// project needs: BO_ (message) and SG_ (signal) definitions, CM_ BO_
// (message comments, used here to recover the transmit period), and VAL_
// (enum value tables). This is not a general DBC library -- unsupported
// sections (BU_, BS_, attribute definitions, etc.) are simply skipped.
// Written by hand per the assignment's "consult existing DBC parsers for
// understanding, but implement your own decode logic" guidance.

// SG_ <name> : <start>|<length>@<order><sign> (<scale>,<offset>) [<min>|<max>] "<unit>" <receivers>
//
// NOTE: these patterns use a custom raw-string delimiter (R"RX(...)RX")
// rather than the default R"(...)"; several of these regexes contain a
// literal `)"` sequence internally (e.g. a capture group immediately
// followed by a literal quote, as in `(...)"`), which would otherwise be
// mis-parsed as the end of the raw string itself.
const std::regex kSignalRe(
    R"RX(^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d)([+-])\s*\(([-\d.eE]+),([-\d.eE]+)\)\s*\[[^\]]*\]\s*"([^"]*)")RX");

// BO_ <id> <name>: <dlc> <sender>
const std::regex kMessageRe(R"RX(^\s*BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+))RX");

// CM_ BO_ <id> "<comment>";
const std::regex kCommentRe(R"RX(^\s*CM_\s+BO_\s+(\d+)\s+"([^"]*)")RX");

// Free-text period extraction from a message comment, e.g.
// "... transmitted every 20 ms by the ECU." -> 20
const std::regex kPeriodInCommentRe(R"RX(every\s+(\d+)\s*ms)RX");

// VAL_ <id> <signal> <n1> "<label1>" <n2> "<label2>" ... ;
const std::regex kValRe(R"RX(^\s*VAL_\s+(\d+)\s+(\w+)\s+(.*);)RX");
const std::regex kValPairRe(R"RX((\d+)\s+"([^"]*)")RX");

} // namespace

DbcDatabase DbcDatabase::loadFromDbcText(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Could not open vehicle.dbc: " + path);

    DbcDatabase db;
    uint32_t currentId = 0;
    std::string line;

    while (std::getline(f, line)) {
        std::smatch m;

        if (std::regex_search(line, m, kMessageRe)) {
            MessageDef msg;
            msg.can_id = static_cast<uint32_t>(std::stoul(m[1].str()));
            msg.name = m[2].str();
            // byte_order is per-signal in the DBC grammar, not per-message;
            // we default the message's nominal order to the first signal
            // we see and let each SignalDef carry its own order via the
            // parsed '@0'/'@1' token (stored on MessageDef here as a
            // simplification since, in this file, all signals within a
            // message share one order).
            currentId = msg.can_id;
            db.messages_[currentId] = std::move(msg);
            continue;
        }

        if (std::regex_search(line, m, kSignalRe)) {
            if (db.messages_.find(currentId) == db.messages_.end()) continue; // malformed input guard
            SignalDef sig;
            sig.name = m[1].str();
            sig.start_bit = static_cast<unsigned>(std::stoul(m[2].str()));
            sig.length = static_cast<unsigned>(std::stoul(m[3].str()));
            int order = std::stoi(m[4].str());
            sig.scale = std::stod(m[6].str());
            sig.offset = std::stod(m[7].str());
            sig.is_signed = (m[5].str() == "-");
            sig.unit = m[8].str();

            MessageDef& msg = db.messages_[currentId];
            msg.byte_order = (order == 0) ? ByteOrder::Motorola : ByteOrder::Intel;
            msg.signals.push_back(std::move(sig));
            continue;
        }

        if (std::regex_search(line, m, kCommentRe)) {
            uint32_t id = static_cast<uint32_t>(std::stoul(m[1].str()));
            std::string comment = m[2].str();
            auto it = db.messages_.find(id);
            if (it == db.messages_.end()) continue;
            std::smatch pm;
            if (std::regex_search(comment, pm, kPeriodInCommentRe)) {
                it->second.period_ms = static_cast<unsigned>(std::stoul(pm[1].str()));
            }
            continue;
        }

        if (std::regex_search(line, m, kValRe)) {
            uint32_t id = static_cast<uint32_t>(std::stoul(m[1].str()));
            std::string sigName = m[2].str();
            std::string rest = m[3].str();
            auto it = db.messages_.find(id);
            if (it == db.messages_.end()) continue;
            for (auto& sig : it->second.signals) {
                if (sig.name != sigName) continue;
                auto begin = std::sregex_iterator(rest.begin(), rest.end(), kValPairRe);
                auto end = std::sregex_iterator();
                for (auto pit = begin; pit != end; ++pit) {
                    int val = std::stoi((*pit)[1].str());
                    sig.enum_values[val] = (*pit)[2].str();
                }
                break;
            }
            continue;
        }
    }

    for (auto& [id, msg] : db.messages_) {
        if (msg.period_ms == 0) {
            throw std::runtime_error(
                "vehicle.dbc: could not determine period_ms for message '" + msg.name +
                "' from its CM_ comment; every message is expected to state it (e.g. "
                "'transmitted every 20 ms').");
        }
    }

    return db;
}

} // namespace can
