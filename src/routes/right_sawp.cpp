#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> sawp_paths = {};

void sawp() {

    auto push_alliance = new MoveTime(6000, 6000, 200);
    push_alliance->add_event(ConditionalEvent{
        []() { return true; },
        []() { descore_piston.set_value(true); intake.set(4000, 12000); }
    });
    queue_motion(push_alliance);
    queue_motion(new Delay(100));
    queue_motion(new MovePoint({47, -48}, 1200, {.reverse = true, .cutoff = 1.0, .drive_max_volt_pct = 75}));
    auto turn_to_loader_1 = new TurnHeading(180, 600, {.cutoff = 5.0});
    turn_to_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(turn_to_loader_1);
    queue_motion(new MovePoint({47, -64}, 1000, {.cutoff = 6.0, .drive_max_volt_pct = 50, .min_volt_pct = 20}));
    queue_motion(new MoveTime(6000, 6000, 500));
    auto score_loader_1 = new MovePoint({48, -20}, 1500, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({48, -24}) < 8.0; },
        []() { intake.set(12000, 12000); lock_piston.set_value(true); }
    });
    queue_motion(score_loader_1);
    queue_motion(new Delay(300));
    auto turn_to_block_group_1 = new SwingHeading(-80, 600, {.locked_side = Side::RIGHT, .cutoff = 5.0});
    turn_to_block_group_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(false); }
    });
    queue_motion(turn_to_block_group_1);
    auto block_group_1 = new MovePoint({24, -24}, 2000, {.drive_max_volt_pct = 50});
    block_group_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(4000, 12000); lock_piston.set_value(false); }
    });
    block_group_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_1);
    auto block_group_2 = new MovePoint({-24, -24}, 2000, {.drive_max_volt_pct = 50});
    block_group_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(false); }
    });
    block_group_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_2);

    queue_motion(new TurnHeading({-135, 300}));
    queue_motion(new Delay(200));
    auto score_mid_goal = new MovePoint({-8, -10}, 1000, {.reverse = true, .drive_max_volt_pct = 50});
    score_mid_goal->add_event(ConditionalEvent{
        []() { return true; },
        []() { middle_piston.set_value(true); }
    });
    score_mid_goal->add_event(ConditionalEvent{
        [score_mid_goal]() { return pros::millis() - score_mid_goal->start_time > 200; },
        []() { intake.set(-12000, -6000); }
    });
    score_mid_goal->add_event(ConditionalEvent{
        [score_mid_goal]() { return pros::millis() - score_mid_goal->start_time > 500; },
        []() { intake.set(12000, 12000); }
    });
    queue_motion(score_mid_goal);
    queue_motion(new TurnHeading({-135, 300}));
    queue_motion(new Delay(300));
    auto move_loader_2 = new MovePoint({-47, -48}, 1000, {.cutoff = 1.0, .drive_max_volt_pct = 75});
    move_loader_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(12000, 0); }
    });
    queue_motion(move_loader_2);
    auto turn_loader_2 = new TurnHeading(180, 400, {.cutoff = 5.0});
    turn_loader_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { middle_piston.set_value(false); intake.set(4000, 12000); }
    });
    queue_motion(turn_loader_2);
    queue_motion(new MovePoint({-47, -64}, 1000, {.cutoff = 6.0, .drive_max_volt_pct = 50, .min_volt_pct = 20}));
    queue_motion(new MoveTime(6000, 6000, 500));
    auto score_loader_2 = new MovePoint({-48, -20}, 1500, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-48, -24}) < 8.0; },
        []() { intake.set(12000, 12000); lock_piston.set_value(true); }
    });
    queue_motion(score_loader_2);

}