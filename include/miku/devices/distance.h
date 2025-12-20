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
    // Sensor offsets are in the robot body frame. Conventions used across the codebase:
    //  - offset_y is the forward offset (positive = forward)
    //  - offset_x is the lateral offset (positive = left)
    //
    // get_expected_reading() uses the following transform to get the sensor position in world frame:
    //   sensor_x = robot_x + offset_x * sin(theta) + offset_y * cos(theta)
    //   sensor_y = robot_y - offset_x * cos(theta) + offset_y * sin(theta)
    // Note: the above uses the project's axis convention where +Y is forward and +X is to the left.
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