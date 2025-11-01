#pragma once
#include "pros/distance.hpp"

enum class Orientation {
    LEFT,
    RIGHT,
    FRONT,
    BACK
};

namespace miku {
class Distance : public pros::Distance {
    double data = -1;
    bool valid = false;
    bool enabled = true;
public:
    double offset_x, offset_y;
    Orientation orientation;
    Distance(const std::uint8_t port, double offset_x, double offset_y, Orientation orientation) 
        : pros::Distance(port), offset_x(offset_x), offset_y(offset_y), orientation(orientation) {}
    void set_enabled(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    double get_reading() { return data; }
};
}