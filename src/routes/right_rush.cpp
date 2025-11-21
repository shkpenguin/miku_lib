#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> rush_paths = {};

void right_rush() {
    auto block_group_rush = new MovePoint({24, -24}, 1500);
    block_group_rush->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    block_group_rush->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_rush);
    queue_motion(new TurnHeading(135, 400));
    queue_motion(new MovePoint({48, -48}, 800));
    queue_motion(new TurnHeading(180, 300));
    queue_motion(new MovePoint({47, -63}, 1000, {.max_speed = 6000}));
    queue_motion(new Delay(300));
    auto score_loader_rush = new MovePoint({48, -20}, 1500, {.reverse = true, .max_speed = 6000});
    score_loader_rush->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(score_loader_rush);
    queue_motion(new Delay(500));
    auto move_from_goal = new MovePoint({48, -40}, 500);
    move_from_goal->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); loader_piston.set_value(false); }
    });
    queue_motion(move_from_goal);
    queue_motion(new TurnHeading(-135, 300));
    queue_motion(new MovePoint({58, -30}, 700, {.reverse = true}));
    queue_motion(new TurnHeading(180, 300));
    queue_motion(new MovePoint({58, -12}, 1500, {.reverse = true, .min_speed = 6000}));
}