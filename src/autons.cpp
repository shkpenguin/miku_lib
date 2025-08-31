#include "mp.h"
#include "motions.h"
#include "main.h"
#include <vector>

std::vector<ControlPoint> test_cp = {
    {24, -48, 0},
    {24, -24, 20},
    // {24, 24, 30},
    {0, 48, 20},
    {0, 48, 20}
};

BezierPath test_path(test_cp);
std::vector<Waypoint> test_waypoints;

void test() {
    test_path.calculate_waypoints();
    test_waypoints = test_path.get_waypoints();
    ramsete(test_waypoints, 20000);
    // // turn_heading(90, 10000);
    // move_point({0, -24}, 10000);
}

void autonomous() {
    setPose({24, -48, 0});
    test();
}