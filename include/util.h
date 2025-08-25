#pragma once

#include <cmath>
#include "config.h"

inline double clamp(double value, double min, double max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

inline double clamp_field(double value) {
    return clamp(value, -72.0, 72.0);
}

inline double sign(double value) {
    return (value > 0) - (value < 0);
}

inline double project(double px, double py, double ox, double oy, double heading) {
    double dx = px - ox;
    double dy = py - oy;
    return dx * cos(heading) + dy * sin(heading);
}

inline double wrap_angle(double angle, double range = 2.0 * M_PI) {
    angle = std::fmod(angle + range / 2.0, range);
    if (angle < 0) angle += range;
    return angle - range / 2.0;
}

inline double dist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

inline double angle_error(double target, double current) {
    return wrap_angle(target - current, 2.0 * M_PI);
}

void move_motors(double l, double r, bool reversed = false) {
    if (reversed) {
        left_motors.move_voltage(-l);
        right_motors.move_voltage(-r);
    } else {
        left_motors.move_voltage(l);
        right_motors.move_voltage(r);
    }
}

void stop_motors() {
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
}

void set_drive_brake(pros::motor_brake_mode_e mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}