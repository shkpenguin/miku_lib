#pragma once

#include <vector>

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

double get_t_param(std::vector<std::vector<double>> P);