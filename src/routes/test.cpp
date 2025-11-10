#include "miku/miku-api.h"

void test() {
    MovePoint move_loader = MovePoint(Point(0, 24), 1000, MovePointParams());
    move_loader.add_event(ConditionalEvent(
        []() { return Miku.get_pose().distance_to({0, 24}) < 10; },
        []() { loader_piston.set_value(true); }
    ));
    queue_motion(&move_loader);
}