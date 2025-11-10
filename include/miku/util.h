#pragma once

#include <cmath>
#include "miku/geometry.h"

#define HALF_FIELD 71.0

inline double clamp(double value, double min, double max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

inline double clamp_field(double value) {
    return clamp(value, -HALF_FIELD, HALF_FIELD);
}

inline double sign(double value) {
    return (value > 0) - (value < 0);
}

namespace miku {
    inline standard_radians atan2(double y, double x) {
        return standard_radians(std::atan2(y, x));
    }
}

inline double project(double px, double py, double ox, double oy, double heading) {
    double dx = px - ox;
    double dy = py - oy;
    return dx * cos(heading) + dy * sin(heading);
}

inline double dist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

inline double ema(double current, double previous, double alpha) {
    return alpha * current + (1 - alpha) * previous;
}

inline int find_quadrant(Pose robot_pose) {
    if(robot_pose.x >= 0 && robot_pose.y >= 0) return 1;
    if(robot_pose.x < 0 && robot_pose.y >= 0) return 2;
    if(robot_pose.x < 0 && robot_pose.y < 0) return 3;
    return 4;
}

// bound value between 0 and mod-1
inline void increment_mod(int& value, int mod) {
    value = (value + 1) % mod;
}

inline void decrement_mod(int& value, int mod) {
    value = (value - 1 + mod) % mod;
}

template<typename T>
class List {
    std::vector<T> values;
    int size;
    int index = 0;
    public:
    List(std::vector<T> vals) : values(vals), size(vals.size()) {}
    List(std::initializer_list<T> vals) : values(vals), size(vals.size()) {}
    T cycle_forward() {
        index = (index + 1) % size;
        return values[index];
    }
    T cycle_reverse() {
        index = (index - 1 + size) % size;
        return values[index];
    }
    T get_value() const {
        return values[index];
    }
};