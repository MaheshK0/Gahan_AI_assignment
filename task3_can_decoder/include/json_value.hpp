#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace can {

// A minimal, hand-written JSON parser -- just enough to read the flat,
// well-formed dbc.json signal database used by this task (objects,
// strings, numbers, nested objects; no arrays are needed for this
// schema). This is not a general-purpose JSON library; per the
// assignment's guidance to avoid third-party dependencies and demonstrate
// the underlying logic, a small recursive-descent parser is written here
// rather than pulling in nlohmann/json or similar.
class JsonValue {
public:
    enum class Type { Null, Object, String, Number, Bool };

    static JsonValue parse(const std::string& text);

    Type type() const { return type_; }
    bool isObject() const { return type_ == Type::Object; }

    // Object accessors.
    bool has(const std::string& key) const;
    const JsonValue& at(const std::string& key) const;
    const std::map<std::string, JsonValue>& members() const { return object_; }

    // Scalar accessors.
    std::string asString() const { return string_; }
    double asDouble() const { return number_; }
    long asLong() const { return static_cast<long>(number_); }
    bool asBool() const { return bool_; }

    // --- Construction helpers used only by the parser implementation.
    // Public (rather than friended to an anonymous-namespace class, which
    // doesn't work portably across translation units) but not meant for
    // general use outside json_value.cpp.
    static JsonValue makeObject() { JsonValue v; v.type_ = Type::Object; return v; }
    static JsonValue makeString(std::string s) { JsonValue v; v.type_ = Type::String; v.string_ = std::move(s); return v; }
    static JsonValue makeNumber(double n) { JsonValue v; v.type_ = Type::Number; v.number_ = n; return v; }
    static JsonValue makeBool(bool b) { JsonValue v; v.type_ = Type::Bool; v.bool_ = b; return v; }
    void insert(const std::string& key, JsonValue val) { object_.emplace(key, std::move(val)); }

private:
    Type type_ = Type::Null;
    std::map<std::string, JsonValue> object_;
    std::string string_;
    double number_ = 0.0;
    bool bool_ = false;
};

} // namespace can
