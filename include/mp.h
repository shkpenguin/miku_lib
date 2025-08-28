#pragma once

#include <vector>
#include "misc.h"

#define MAX_RPM 600
#define GEAR_RATIO 1.0
#define CIRC (2.75 * M_PI)
#define TRACK_WIDTH 12.0

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
    ControlPoint(double x = 0.0, double y = 0.0, double velocity = 0.0) 
        : x(x), y(y), velocity(velocity) {}
};

double get_t_param(const std::vector<std::vector<double>> P);

struct BezierPath {
    std::vector<ControlPoint> control_points;
    std::vector<Waypoint> waypoints;

    void calculate_waypoints();
    std::vector<Waypoint> get_waypoints() {
        return waypoints;
    }

    BezierPath(std::vector<ControlPoint> control_points)
        : control_points(control_points) {}
    BezierPath() = default;
};