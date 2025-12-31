#pragma once
// todo: make new logger

#include <string>
#include <functional>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <type_traits>
#include <utility>

namespace miku {

class Logger {
public:
    // path: file opened in binary append mode; each update() writes one JSON object (newline terminated)
    explicit Logger(const std::string &path) : out(path, std::ios::binary | std::ios::app) {}

    ~Logger() { if (out.is_open()) out.flush(); }

    // store a key with a callable that returns a value (any type). The callable is evaluated during update().
    template <typename F>
    void log(const std::string &key, F &&value_fn) {
        std::lock_guard<std::mutex> lk(mtx_);
        // capture a local copy of the callable and wrap into a string-returning function
        auto fn = std::function<std::string()>(
            [fn_copy = std::forward<F>(value_fn)]() -> std::string {
                try {
                    if constexpr (std::is_same_v<std::invoke_result_t<F>, std::string>) {
                        return fn_copy();
                    } else {
                        std::ostringstream ss;
                        ss << fn_copy();
                        return ss.str();
                    }
                } catch (...) {
                    return std::string();
                }
            });
        entries_.emplace_back(key, std::move(fn));
    }

    // Evaluate all stored pairs, write a single JSON object to the binary file (append), and flush.
    // Format example: {"ts":1623456789012,"key1":"val1","key2":"42"}
    void update() {
        std::vector<std::pair<std::string, std::function<std::string()>>> snapshot;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            snapshot = entries_; // copy for safe evaluation without holding the lock
        }

        std::ostringstream ss;
        ss << '{';
        // timestamp in milliseconds since epoch
        ss << "\"ts\":" << current_ms();

        for (auto &p : snapshot) {
            const auto &k = p.first;
            std::string v;
            try { v = p.second(); } catch(...) { v = ""; }
            ss << ',';
            ss << '\"' << escape(k) << "\":";
            // always write as JSON string for safety
            ss << '\"' << escape(v) << '\"';
        }
        ss << "}\n";

        if (out.is_open()) {
            std::string s = ss.str();
            out.write(s.data(), static_cast<std::streamsize>(s.size()));
            out.flush();
        }
    }

private:
    std::vector<std::pair<std::string, std::function<std::string()>>> entries_;
    std::mutex mtx_;
    std::ofstream out;

    static inline long long current_ms() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

    static std::string escape(const std::string &in) {
        std::string out;
        out.reserve(in.size() + 8);
        for (unsigned char c : in) {
            switch (c) {
                case '\"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\b': out += "\\b";  break;
                case '\f': out += "\\f";  break;
                case '\n': out += "\\n";  break;
                case '\r': out += "\\r";  break;
                case '\t': out += "\\t";  break;
                default:
                    if (c < 0x20 || c == 0x7f) {
                        std::ostringstream ss;
                        ss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)c;
                        out += ss.str();
                    } else {
                        out += static_cast<char>(c);
                    }
            }
        }
        return out;
    }
};

} // namespace miku