#pragma once

#include "pros/rtos.hpp"
#include <string>
#include <vector>
#include <fstream>

#include <cstdint>
#include <cstring>
#include <optional>

enum class FieldType : uint8_t {
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    INT8,
    INT16,
    INT32,
    INT64,
    FLOAT32,
    FLOAT64
};

inline size_t FieldTypeSize(FieldType f) {
    switch (f) {
        case FieldType::UINT8: case FieldType::INT8:    return 1;
        case FieldType::UINT16: case FieldType::INT16:  return 2;
        case FieldType::UINT32: case FieldType::INT32:  return 4;
        case FieldType::UINT64: case FieldType::INT64:  return 8;
        case FieldType::FLOAT32: return 4;
        case FieldType::FLOAT64: return 8;
        default: return 0;
    }
}

struct Field {
    std::string name;
    FieldType type;
    int repeat = 1; // number of repetitions (default 1)

    Field() = default;
    Field(std::string n, FieldType t, int r = 1) : name(std::move(n)), type(t), repeat(r) {}

    size_t size() const { return FieldTypeSize(type) * static_cast<size_t>(repeat); }
};

struct PacketTemplate {
    std::string name;
    std::vector<Field> fields;

    PacketTemplate() = default;
    PacketTemplate(std::string n, std::vector<Field> f) : name(std::move(n)), fields(std::move(f)) {}

    size_t total_size() const {
        size_t s = 0;
        for (auto &f : fields) s += f.size();
        return s;
    }
};

// Binary logger that writes a human-readable header file (.hdr) listing templates
// and a binary stream (.bin). Each binary packet is written in little-endian raw
// bytes using the layout: [uint64_t timestamp_ms][uint8_t template_id][payload bytes].
// Offsets reported in the .hdr file are relative to the payload start.
class BinaryLogger {
public:
    explicit BinaryLogger(const std::string &path_base);
    ~BinaryLogger();

    void append_to_header(const std::string &line) {
        if (header_out_.is_open()) {
            header_out_ << line << "\n";
        }
    }

    // Register a template. Returns template id (0-based) on success, or -1 on failure.
    int register_template(const PacketTemplate &pt);

    // Write a payload (raw bytes) for the template id. Returns true on success.
    bool write_packet(int template_id, const std::vector<uint8_t> &payload);

    // Convenience: pack trivially-copyable scalars in order. The pack must match the
    // template exactly in total size.
    template <typename... Ts>
    bool write_packet(int template_id, Ts... values) {
        static_assert(std::conjunction_v<std::is_trivially_copyable<Ts>...>, "Args must be trivially copyable");
        std::vector<uint8_t> buf;
        buf.reserve(2048); 
        (append_scalar(buf, values), ...);
        return write_packet(template_id, buf);
    }

    // Flush data to disk
    void flush();

private:
    std::string base_path_;
    std::ofstream data_out_;   // binary .bin file (append)
    std::ofstream header_out_; // text .hdr file (append)
    pros::Mutex mtx_;
    std::vector<PacketTemplate> templates_;

    static void append_pod(std::vector<uint8_t> &buf, const void *p, size_t s) {
        const uint8_t *b = reinterpret_cast<const uint8_t *>(p);
        buf.insert(buf.end(), b, b + s);
    }

    template <typename T>
    static void append_scalar(std::vector<uint8_t> &buf, const T &v) {
        static_assert(std::is_trivially_copyable_v<T>);
        // copy bytes (little-endian expected on target hardware)
        T tmp = v;
        append_pod(buf, &tmp, sizeof(T));
    }
};