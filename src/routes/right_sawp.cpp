#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> sawp_paths = {};

void sawp() {

    auto push_alliance = new MoveTime(6000, 6000, 400);
    push_alliance->add_event(ConditionalEvent{
        []() { return true; },
        []() { descore_piston.set_value(true); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(push_alliance);
    queue_motion(new Delay(100));
    queue_motion(new MovePoint({48, -48}, 1200, {.reverse = true}));
    auto turn_to_loader_1 = new TurnHeading(180, 1000);
    turn_to_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(turn_to_loader_1);
    queue_motion(new MovePoint({48, -63}, 800, {.max_speed = 6000}));
    queue_motion(new Delay(300));
    auto score_loader_1 = new MovePoint({48, -20}, 1500, {.reverse = true, .max_speed = 6000});
    score_loader_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({48, -24}) < 5.0; },
        []() { intake_top.move_voltage(12000); lock_piston.set_value(true); }
    });
    queue_motion(score_loader_1);
    queue_motion(new Delay(300));
    auto turn_to_block_group_1 = new SwingHeading(-80, 800, {.locked_side = Side::RIGHT});
    turn_to_block_group_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(false); }
    });
    queue_motion(turn_to_block_group_1);
    auto block_group_1 = new MovePoint({24, -24}, 2000, {.max_speed = 6000});
    block_group_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake_top.move_voltage(4000); lock_piston.set_value(false); }
    });
    block_group_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_1);
    auto block_group_2 = new MovePoint({-24, -24}, 2000, {.max_speed = 6000});
    block_group_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(false); }
    });
    block_group_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_2);
    queue_motion(new TurnHeading({-135, 400}));
    auto score_mid_goal = new MovePoint({-10, -10}, 1000, {.reverse = true, .max_speed = 6000});
    score_mid_goal->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-10, -10}) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(10000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(score_mid_goal);
    queue_motion(new Delay(300));
    auto move_loader_2 = new MovePoint({-48, -48}, 1000);
    move_loader_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(move_loader_2);
    queue_motion(new TurnHeading(180, 700));
    queue_motion(new MovePoint({-48, -63}, 1000, {.max_speed = 6000}));
    queue_motion(new Delay(300));
    auto score_loader_2 = new MovePoint({-48, -20}, 1500, {.reverse = true, .max_speed = 6000});
    score_loader_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-48, -24}) < 5.0; },
        []() { intake_top.move_voltage(12000); lock_piston.set_value(true); }
    });
    queue_motion(score_loader_2);
    queue_motion(new Delay(300));

}