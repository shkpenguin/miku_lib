#pragma once

#include <vector>
#include <cmath>

struct Waypoint {
    float x;
    float y;
    float dx;
    float dy;
    float ddx;
    float ddy;
    float theta;
    float linvel;
    float angvel;
    float t;

    Waypoint() : x(0), y(0), dx(0), dy(0), ddx(0), ddy(0), theta(0), linvel(0), angvel(0), t(0) {}
};

struct ControlPoint {
    float x;
    float y;
    float velocity;

    ControlPoint(float x, float y, float velocity)
        : x(x), y(y), velocity(velocity) {}
    ControlPoint(float x, float y)
        : x(x), y(y), velocity(0) {}
    ControlPoint() : x(0), y(0), velocity(0) {}
};

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