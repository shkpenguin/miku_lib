#include "mp.h"
#include "motions.h"
#include <vector>

struct Path {
    std::vector<std::vector<Point>> control_points;
    std::vector<double> velocities;

    BezierPath path;
    void init_path() {
        path = BezierPath(control_points, velocities);
    }

    void load_path() {
        path.calculate_waypoints();
    }

    Path(std::vector<std::vector<Point>> control_points,
         std::vector<double> velocities)
        : control_points(control_points), velocities(velocities) {
            init_path();
        }
};

std::vector<Path> test_paths = {
    {
        {
            { {0, 0}, {24, 0}, {24, 24}, {48, 24} }
        },
        {50, 50, 0, 0}
    }
};

void test() {
    for (auto &p : test_paths) {
        p.init_path();
        ramsete(p.path.get_waypoints(), 2.0, 0.7, 1.0, 1.0, 0.0, 0.0, false, 0);
    }

    
}