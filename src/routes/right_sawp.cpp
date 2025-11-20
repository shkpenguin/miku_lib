#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> sawp_paths = {};

void sawp() {
    auto push_alliance = new MovePoint({-6, -48}, 1500, {.reverse = true});
    push_alliance->add_event(ConditionalEvent{
        []() { return true; },
        []() { descore_piston.set_value(true); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(push_alliance);
    queue_motion(new MovePoint({48, -48}, 2000, {.reverse = true}));
    auto turn_to_loader_1 = new TurnHeading(90, 1000);
    turn_to_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(turn_to_loader_1);
    queue_motion(new MovePoint({48, -67}, 500));
    queue_motion(new Delay(500));
    queue_motion(new MovePoint({48, -20}, 1500, {.reverse = true}));

}