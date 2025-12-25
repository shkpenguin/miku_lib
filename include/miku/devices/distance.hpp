#pragma once
#include "pros/distance.hpp"
#include <cmath>

// Sensor orientation is now stored as an angle (radians):
//  0 = forward (+Y), positive = CCW


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

    // Orientation stored as angle in radians (0 = front/+Y). Precompute cos/sin for efficiency.
    float orientation_angle = 0.0f;
    float cos_orient = 1.0f;
    float sin_orient = 0.0f;

    Distance(const std::uint8_t port, float offset_x, float offset_y, float orientation_angle)
        : pros::Distance(port), offset_x(offset_x), offset_y(offset_y) {
        set_orientation_angle(orientation_angle);
    }

    void set_orientation_angle(float ang_rad) {
        orientation_angle = ang_rad;
        cos_orient = std::cos(ang_rad);
        sin_orient = std::sin(ang_rad);
    }

    void set_enabled(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    float get_reading() { return data; }
};
}