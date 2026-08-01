#include "json_lite.hpp"

#include <cctype>
#include <stdexcept>

namespace can {

JsonValue JsonValue::parse(const std::string& text) {
    size_t pos = 0;
    return parseValue(text, pos);
}

void JsonValue::skipWs(const std::string& s, size_t& pos) {
    while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
}

void JsonValue::expect(const std::string& s, size_t& pos, char c) {
    skipWs(s, pos);
    if (pos >= s.size() || s[pos] != c) {
        throw std::runtime_error(std::string("Expected '") + c + "' in JSON at position " + std::to_string(pos));
    }
    ++pos;
}

JsonValue JsonValue::parseValue(const std::string& s, size_t& pos) {
    skipWs(s, pos);
    if (pos >= s.size()) throw std::runtime_error("Unexpected end of JSON");
    char c = s[pos];
    if (c == '{') return parseObject(s, pos);
    if (c == '"') return parseString(s, pos);
    if (c == 't' || c == 'f') return parseBool(s, pos);
    if (c == 'n') return parseNull(s, pos);
    return parseNumber(s, pos);
}

JsonValue JsonValue::parseObject(const std::string& s, size_t& pos) {
    JsonValue v;
    v.type_ = Type::Object;
    expect(s, pos, '{');
    skipWs(s, pos);
    if (pos < s.size() && s[pos] == '}') { ++pos; return v; }
    while (true) {
        skipWs(s, pos);
        JsonValue key = parseString(s, pos);
        skipWs(s, pos);
        expect(s, pos, ':');
        JsonValue val = parseValue(s, pos);
        v.object_[key.string_] = std::move(val);
        skipWs(s, pos);
        if (pos < s.size() && s[pos] == ',') { ++pos; continue; }
        expect(s, pos, '}');
        break;
    }
    return v;
}

JsonValue JsonValue::parseString(const std::string& s, size_t& pos) {
    JsonValue v;
    v.type_ = Type::String;
    expect(s, pos, '"');
    std::string out;
    while (pos < s.size() && s[pos] != '"') {
        char c = s[pos++];
        if (c == '\\' && pos < s.size()) {
            char esc = s[pos++];
            switch (esc) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                default: out += esc; break;
            }
        } else {
            out += c;
        }
    }
    expect(s, pos, '"');
    v.string_ = std::move(out);
    return v;
}

JsonValue JsonValue::parseBool(const std::string& s, size_t& pos) {
    JsonValue v;
    v.type_ = Type::Bool;
    if (s.compare(pos, 4, "true") == 0) { v.bool_ = true; pos += 4; }
    else if (s.compare(pos, 5, "false") == 0) { v.bool_ = false; pos += 5; }
    else throw std::runtime_error("Invalid boolean literal in JSON");
    return v;
}

JsonValue JsonValue::parseNull(const std::string& s, size_t& pos) {
    JsonValue v;
    v.type_ = Type::Null;
    if (s.compare(pos, 4, "null") == 0) pos += 4;
    else throw std::runtime_error("Invalid null literal in JSON");
    return v;
}

JsonValue JsonValue::parseNumber(const std::string& s, size_t& pos) {
    JsonValue v;
    v.type_ = Type::Number;
    size_t start = pos;
    if (pos < s.size() && (s[pos] == '-' || s[pos] == '+')) ++pos;
    while (pos < s.size() &&
           (std::isdigit(static_cast<unsigned char>(s[pos])) || s[pos] == '.' ||
            s[pos] == 'e' || s[pos] == 'E' || s[pos] == '-' || s[pos] == '+')) {
        ++pos;
    }
    if (pos == start) throw std::runtime_error("Invalid number in JSON");
    v.number_ = std::stod(s.substr(start, pos - start));
    return v;
}

bool JsonValue::has(const std::string& key) const {
    return type_ == Type::Object && object_.count(key) > 0;
}

const JsonValue& JsonValue::at(const std::string& key) const {
    auto it = object_.find(key);
    if (it == object_.end()) throw std::runtime_error("Missing JSON key: " + key);
    return it->second;
}

double JsonValue::asNumber() const {
    if (type_ != Type::Number) throw std::runtime_error("JSON value is not a number");
    return number_;
}

bool JsonValue::asBool() const {
    if (type_ != Type::Bool) throw std::runtime_error("JSON value is not a bool");
    return bool_;
}

const std::string& JsonValue::asString() const {
    if (type_ != Type::String) throw std::runtime_error("JSON value is not a string");
    return string_;
}

} // namespace can
