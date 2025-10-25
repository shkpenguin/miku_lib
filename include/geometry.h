#pragma once

#include <cmath>
#include <vector>

struct Point {
    double x;
    double y;

    Point(double x = 0.0, double y = 0.0) 
        : x(x), y(y) {}

    double magnitude() const {
        return std::hypot(x, y);
    }
};

struct Pose {
    double x; 
    double y;
    double theta; 

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0, bool radians = true) 
        : x(x), y(y), theta(radians ? theta : theta * M_PI / 180.0) {}
    Pose(Point position, double theta = 0.0, bool radians = true) 
        : x(position.x), y(position.y), theta(radians ? theta : theta * M_PI / 180.0) {}

    double magnitude() const {
        return std::hypot(x, y);
    }

    double angle_to(const Point& other) const {
        return atan2(other.y - y, other.x - x);
    }
};

struct Polygon {
    std::vector<Point> ccw_vertices;
};