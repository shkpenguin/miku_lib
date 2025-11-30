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
    float data = -1;
    bool valid = false;
    bool enabled = true;
public:
    float offset_x, offset_y;
    Orientation orientation;
    Distance(const std::uint8_t port, float offset_x, float offset_y, Orientation orientation) 
        : pros::Distance(port), offset_x(offset_x), offset_y(offset_y), orientation(orientation) {}
    void set_enabled(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    float get_reading() { return data; }
};
}