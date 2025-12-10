#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> rush_paths = {};

void right_rush() {
    auto block_group_rush = new MovePoint({24, -24}, 1500);
    block_group_rush->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(4000, 12000); }
    });
    block_group_rush->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_rush);
    queue_motion(new TurnHeading(135, 400, {.cutoff = 5.0}));
    queue_motion(new MovePoint({48, -48}, 800, {.cutoff = 3.0}));
    queue_motion(new TurnHeading(180, 300, {.cutoff = 5.0}));
    queue_motion(new MovePoint({47, -64}, 1000));
    queue_motion(new MoveTime(3000, 3000, 100));
    auto score_loader_rush = new MovePoint({48, -20}, 1500, {.reverse = true, .drive_max_volt_pct = 50});
    score_loader_rush->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set(12000, 12000); }
    });
    queue_motion(score_loader_rush);
    queue_motion(new Delay(500));   

    // illegal wing push (now legal)
    auto move_from_goal = new MovePoint({48, -40}, 500);
    move_from_goal->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); loader_piston.set_value(false); }
    });
    queue_motion(move_from_goal);
    queue_motion(new TurnHeading(-135, 300));
    queue_motion(new MovePoint({56, -30}, 700, {.reverse = true}));
    queue_motion(new TurnHeading(180, 300));
    queue_motion(new MovePoint({57, -10}, 1500, {.reverse = true, .cutoff = 5.0, .min_volt_pct = 75}));
    auto hold_wing = new MoveTime(0, 0, 500); 
    hold_wing->add_event(ConditionalEvent{
        []() { return true; },
        []() { Miku.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); }
    });
    queue_motion(hold_wing);
}