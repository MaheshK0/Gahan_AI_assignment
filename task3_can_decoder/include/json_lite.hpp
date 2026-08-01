#pragma once
#include <map>
#include <string>
#include <vector>

namespace can {

// A minimal, hand-rolled JSON parser -- intentionally NOT a general-purpose
// JSON library. It supports exactly what dbc.json needs: nested objects,
// strings, numbers, booleans, and null. No arrays are present in dbc.json,
// so array support is omitted rather than half-implemented. Written by
// hand per the assignment's "avoid third-party libraries, demonstrate the
// underlying logic" guidance, rather than pulling in nlohmann/json or
// similar for what is a small, fixed-shape config file.
class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Object };

    JsonValue() : type_(Type::Null) {}
    static JsonValue parse(const std::string& text);

    Type type() const { return type_; }
    bool isObject() const { return type_ == Type::Object; }

    // Object accessors.
    bool has(const std::string& key) const;
    const JsonValue& at(const std::string& key) const;
    const std::map<std::string, JsonValue>& members() const { return object_; }

    double asNumber() const;
    bool asBool() const;
    const std::string& asString() const;

private:
    Type type_;
    double number_ = 0;
    bool bool_ = false;
    std::string string_;
    std::map<std::string, JsonValue> object_;

    // Recursive-descent parsing, implemented as private static members
    // (rather than a separate friend class) so there's no namespace/
    // friend-declaration mismatch to keep in sync.
    static JsonValue parseValue(const std::string& s, size_t& pos);
    static JsonValue parseObject(const std::string& s, size_t& pos);
    static JsonValue parseString(const std::string& s, size_t& pos);
    static JsonValue parseBool(const std::string& s, size_t& pos);
    static JsonValue parseNull(const std::string& s, size_t& pos);
    static JsonValue parseNumber(const std::string& s, size_t& pos);
    static void skipWs(const std::string& s, size_t& pos);
    static void expect(const std::string& s, size_t& pos, char c);
};

} // namespace can
