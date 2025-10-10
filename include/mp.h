#pragma once

#include <vector>
#include <cmath>
#include "config.h"

#define MAX_RPM 600
#define CIRC (WHEEL_DIAMETER * M_PI)

struct Waypoint {
    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double theta;
    double linvel;
    double angvel;
    double t;

    Waypoint() : x(0), y(0), dx(0), dy(0), ddx(0), ddy(0), theta(0), linvel(0), angvel(0), t(0) {}
};

struct ControlPoint {
    double x;
    double y;
    double velocity;

    ControlPoint(double x, double y, double velocity)
        : x(x), y(y), velocity(velocity) {}
    ControlPoint(double x, double y)
        : x(x), y(y), velocity(0) {}
    ControlPoint() : x(0), y(0), velocity(0) {}
};

double get_t_param(const std::vector<std::vector<double>> P);

struct BezierPath {
    std::vector<ControlPoint> control_points;
    std::vector<Waypoint> waypoints;

    void calculate_waypoints();
    std::vector<Waypoint> get_waypoints() {
        return waypoints;
    }

    BezierPath(std::initializer_list<ControlPoint> points)
        : control_points(points) {}
    BezierPath(std::vector<ControlPoint> control_points)
        : control_points(control_points) {}
    BezierPath(const BezierPath& other) 
        : control_points(other.control_points), waypoints(other.waypoints) {}
    BezierPath() = default;
};