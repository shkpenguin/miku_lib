#include "miku/miku-api.h"
#include "miku/motions.h"

BezierPath test_path({
    {24, -24, 0},
    {24, -24, 50},
    {24, 0, 25},
    {18, 18, 25},
    {0, 24, 25},
    {-18, 18, 25},
    {-24, 0, 25},
    {-24, -24, 0},
    {-24, -24, 0}
});

std::vector<std::reference_wrapper<BezierPath>> test_paths = {std::ref(test_path)};

void test() {
    queue_motion(new TurnHeading(90, 10000));
    queue_motion(new MovePose({24, -24}, 0, 10000));
    test_path.calculate_waypoints();
    queue_motion(new Ramsete(test_path.get_waypoints(), 10000));
    // queue_motion(new TurnHeading(90, 10000));
}

