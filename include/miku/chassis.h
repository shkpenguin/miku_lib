#pragma once

#include "miku-api.h"
#include <memory>

namespace miku {

class Chassis {
public:
    Chassis();
    Chassis(std::shared_ptr<MotorGroup> left, std::shared_ptr<MotorGroup> right)
        : left_motors(left), right_motors(right) {}

    void set_voltages(double left_speed, double right_speed) {
        left_motors->move_voltage(left_speed);
        right_motors->move_voltage(right_speed);
    }
    void set_velocities(double left_velocity, double right_velocity) {
        left_motors->move_velocity(left_velocity);
        right_motors->move_velocity(right_velocity);
    }
    void stop() {
        left_motors->move_voltage(0);
        right_motors->move_voltage(0);
    }
    void set_brake_mode(pros::motor_brake_mode_e mode) {
        left_motors->set_brake_mode(mode);
        right_motors->set_brake_mode(mode);
    }

    inline Pose get_pose();
    inline Pose get_pose_delta();
    inline Point get_position();
    inline Point get_position_delta();
    inline standard_radians get_heading();
    inline standard_radians get_heading_delta();
    inline void set_pose(Pose new_pose);
    inline void reset(Pose initial_pose);

    void update_odometry();

private:
    std::shared_ptr<MotorGroup> left_motors;
    std::shared_ptr<MotorGroup> right_motors;
    Pose pose;
    Pose pose_delta;
};

} // namespace miku