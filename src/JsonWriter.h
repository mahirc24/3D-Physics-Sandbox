// JsonWriter.h — a tiny, dependency-free JSON object serialiser.
//
// We only ever emit flat-ish objects (numbers, strings, nested objects, and
// arrays of numbers), so a full JSON library would be overkill. This keeps the
// C++ side dependency-free while still handing the Python harness clean JSON.
#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace pbx {

class JsonWriter {
public:
    JsonWriter() { os_ << "{"; }

    JsonWriter& kv(const std::string& key, double value) {
        sep();
        os_ << quote(key) << ':' << num(value);
        return *this;
    }
    JsonWriter& kv(const std::string& key, long long value) {
        sep();
        os_ << quote(key) << ':' << value;
        return *this;
    }
    JsonWriter& kv(const std::string& key, int value) {
        return kv(key, static_cast<long long>(value));
    }
    JsonWriter& kv(const std::string& key, bool value) {
        sep();
        os_ << quote(key) << ':' << (value ? "true" : "false");
        return *this;
    }
    JsonWriter& kv(const std::string& key, const std::string& value) {
        sep();
        os_ << quote(key) << ':' << quote(value);
        return *this;
    }

    // Nested raw object/array already formatted as a JSON string.
    JsonWriter& raw(const std::string& key, const std::string& jsonValue) {
        sep();
        os_ << quote(key) << ':' << jsonValue;
        return *this;
    }

    JsonWriter& array(const std::string& key, const std::vector<double>& xs) {
        sep();
        os_ << quote(key) << ":[";
        for (size_t i = 0; i < xs.size(); ++i) {
            if (i) os_ << ',';
            os_ << num(xs[i]);
        }
        os_ << ']';
        return *this;
    }

    std::string str() {
        return os_.str() + "}";
    }

private:
    void sep() {
        if (!first_) os_ << ',';
        first_ = false;
    }
    static std::string num(double v) {
        std::ostringstream s;
        s.precision(9);
        s << v;
        return s.str();
    }
    static std::string quote(const std::string& s) {
        std::string out = "\"";
        for (char c : s) {
            switch (c) {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n";  break;
                case '\t': out += "\\t";  break;
                default:   out += c;      break;
            }
        }
        out += '"';
        return out;
    }

    std::ostringstream os_;
    bool first_ = true;
};

}  // namespace pbx
