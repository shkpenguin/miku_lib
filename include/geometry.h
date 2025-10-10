#pragma once

#include <cmath>

struct Point {
    double x;
    double y;

    Point(double x = 0.0, double y = 0.0) 
        : x(x), y(y) {}
};

struct Pose {
    double x; 
    double y;
    double theta; 

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0, bool radians = true) 
        : x(x), y(y), theta(radians ? theta : theta * M_PI / 180.0) {}
    Pose(Point position, double theta = 0.0, bool radians = true) 
        : x(position.x), y(position.y), theta(radians ? theta : theta * M_PI / 180.0) {}
};