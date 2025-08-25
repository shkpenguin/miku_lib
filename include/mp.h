#pragma once

#include <vector>
#include "config.h"
#include "misc.h"

#define MAX_RPM 600
#define GEAR_RATIO 1.0
#define CIRC (WHEEL_DIAMETER * M_PI)
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

double get_t_param(const std::vector<std::vector<double>> P);

struct BezierPath {
    std::vector<std::vector<Point>> control_points;
    std::vector<Waypoint> waypoints;
    double start_linvel;
    double end_linvel;
    double start_angvel;
    double end_angvel;

    void calculate_waypoints();
    std::vector<Waypoint> get_waypoints() {
        return waypoints;
    }

    BezierPath(std::vector<std::vector<Point>> control_points,
               double start_linvel = 0, double end_linvel = 0,
               double start_angvel = 0, double end_angvel = 0)
        : control_points(control_points),
          start_linvel(start_linvel), end_linvel(end_linvel),
          start_angvel(start_angvel), end_angvel(end_angvel) {}
    BezierPath(std::vector<std::vector<Point>> control_points,
               std::vector<double> velocities)
        : control_points(control_points), 
          start_linvel(velocities[0]), end_linvel(velocities[1]),
          start_angvel(velocities[2]), end_angvel(velocities[3]) {}
    BezierPath() = default;
};