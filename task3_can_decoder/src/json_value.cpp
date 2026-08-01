#include "json_value.hpp"

#include <cctype>
#include <stdexcept>

namespace can {

namespace {

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    JsonValue parseValue() {
        skipWhitespace();
        if (pos_ >= s_.size()) throw std::runtime_error("Unexpected end of JSON");
        char c = s_[pos_];
        if (c == '{') return parseObject();
        if (c == '"') return parseString();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') { pos_ += 4; return JsonValue{}; } // "null"
        return parseNumber();
    }

private:
    const std::string& s_;
    size_t pos_ = 0;

    void skipWhitespace() {
        while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) ++pos_;
    }

    char peek() { skipWhitespace(); return pos_ < s_.size() ? s_[pos_] : '\0'; }

    void expect(char c) {
        skipWhitespace();
        if (pos_ >= s_.size() || s_[pos_] != c) {
            throw std::runtime_error(std::string("Expected '") + c + "' in JSON");
        }
        ++pos_;
    }

    JsonValue parseObject() {
        JsonValue v = JsonValue::makeObject();
        expect('{');
        if (peek() == '}') { ++pos_; return v; }
        while (true) {
            skipWhitespace();
            JsonValue key = parseString();
            expect(':');
            JsonValue val = parseValue();
            v.insert(key.asString(), std::move(val));
            skipWhitespace();
            if (peek() == ',') { ++pos_; continue; }
            expect('}');
            break;
        }
        return v;
    }

    JsonValue parseString() {
        expect('"');
        std::string out;
        while (pos_ < s_.size() && s_[pos_] != '"') {
            char c = s_[pos_++];
            if (c == '\\' && pos_ < s_.size()) {
                char esc = s_[pos_++];
                switch (esc) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    default: out += esc; break;
                }
            } else {
                out += c;
            }
        }
        expect('"');
        return JsonValue::makeString(out);
    }

    JsonValue parseNumber() {
        size_t start = pos_;
        if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) ++pos_;
        while (pos_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '.' ||
                s_[pos_] == 'e' || s_[pos_] == 'E' || s_[pos_] == '-' || s_[pos_] == '+')) {
            ++pos_;
        }
        if (pos_ == start) throw std::runtime_error("Invalid number in JSON");
        return JsonValue::makeNumber(std::stod(s_.substr(start, pos_ - start)));
    }

    JsonValue parseBool() {
        if (s_.compare(pos_, 4, "true") == 0) { pos_ += 4; return JsonValue::makeBool(true); }
        if (s_.compare(pos_, 5, "false") == 0) { pos_ += 5; return JsonValue::makeBool(false); }
        throw std::runtime_error("Invalid literal in JSON");
    }
};

} // namespace

JsonValue JsonValue::parse(const std::string& text) {
    Parser p(text);
    return p.parseValue();
}

bool JsonValue::has(const std::string& key) const {
    return object_.find(key) != object_.end();
}

const JsonValue& JsonValue::at(const std::string& key) const {
    auto it = object_.find(key);
    if (it == object_.end()) throw std::runtime_error("Missing JSON key: " + key);
    return it->second;
}

} // namespace can
