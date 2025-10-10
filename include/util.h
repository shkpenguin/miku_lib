#pragma once

#include <cmath>
#include "geometry.h"

#define HALF_FIELD 71.5

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

inline double ema(double current, double previous, double alpha) {
    return alpha * current + (1 - alpha) * previous;
}

inline int find_quadrant(Pose robot_pose) {
    if(robot_pose.x >= 0 && robot_pose.y >= 0) return 1;
    if(robot_pose.x < 0 && robot_pose.y >= 0) return 2;
    if(robot_pose.x < 0 && robot_pose.y < 0) return 3;
    return 4;
}

enum class Orientation {
    LEFT,
    RIGHT,
    FRONT,
    BACK
};

enum class Side {
    LEFT,
    RIGHT
};

enum MotionType {
    TURN_HEADING,
    TURN_POINT,
    SWING_HEADING,
    SWING_POINT,
    MOVE_POINT,
    MOVE_POSE,
    MOVE_TIME,
    RAMSETE
};