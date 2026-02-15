#include "miku/log.hpp"
#include <mutex>
#include <iostream>
#include <iomanip>
#include <chrono>

BinaryLogger::BinaryLogger(const std::string &path_base)
    : base_path_(path_base)
{
    // open binary file for append
    data_out_.open(base_path_ + ".bin", std::ios::binary | std::ios::app);
    // open header file for append (human readable). If it doesn't exist, we'll create it.
    header_out_.open(base_path_ + ".hdr", std::ios::app);
    if (header_out_.is_open()) {
        header_out_ << "# Packet templates\n";
        header_out_ << "# FORMAT: [u64 timestamp_ms LE][u8 template_id][payload bytes]\n";
        header_out_ << "# Offsets below are relative to payload start\n";
    }
}

BinaryLogger::~BinaryLogger() {
    flush();
    if (data_out_.is_open()) data_out_.close();
    if (header_out_.is_open()) header_out_.close();
}

int BinaryLogger::register_template(const PacketTemplate &pt) {
    std::lock_guard<pros::Mutex> lk(mtx_);
    if (pt.name.empty()) return -1;
    // id is current number of templates
    int id = static_cast<int>(templates_.size());
    templates_.push_back(pt);

    if (header_out_.is_open()) {
        // write brief summary with payload and total sizes
        const size_t payload_size = pt.total_size();
        const size_t total_size = payload_size + sizeof(uint64_t) + 1; // ts + id
        header_out_ << "ID " << id << " NAME \"" << pt.name << "\" PAYLOAD_SIZE " << payload_size << " TOTAL_SIZE " << total_size << "\n";
        // write each field (name,type,size,repeat,offset) - offsets are payload-relative
        size_t offset = 0;
        for (auto &f : pt.fields) {
            header_out_ << "  " << f.name << ", type=" << static_cast<int>(f.type)
                        << ", bytes=" << FieldTypeSize(f.type)
                        << ", repeat=" << f.repeat
                        << ", offset=" << offset << "\n";
            offset += f.size();
        }
    }

    return id;
}

bool BinaryLogger::write_packet(int template_id, const std::vector<uint8_t> &payload) {
    std::lock_guard<pros::Mutex> lk(mtx_);
    if (template_id < 0 || static_cast<size_t>(template_id) >= templates_.size()) return false;
    const auto &pt = templates_[template_id];
    if (payload.size() != pt.total_size()) return false;
    if (!data_out_.is_open()) return false;

    // write timestamp (u64 ms), one-byte id, then payload
    uint64_t ts = pros::millis();
    data_out_.write(reinterpret_cast<const char *>(&ts), static_cast<std::streamsize>(sizeof(ts)));

    uint8_t id = static_cast<uint8_t>(template_id);
    data_out_.write(reinterpret_cast<const char *>(&id), 1);

    if (payload.size()) data_out_.write(reinterpret_cast<const char *>(payload.data()), static_cast<std::streamsize>(payload.size()));
    data_out_.flush();
    return true;
}

void BinaryLogger::flush() {
    std::lock_guard<pros::Mutex> lk(mtx_);
    if (data_out_.is_open()) data_out_.flush();
    if (header_out_.is_open()) header_out_.flush();
}