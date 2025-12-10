#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {};

// void skills() {

//     auto barrier_cross = new Delay(2500);
//     barrier_cross->add_event(ConditionalEvent{
//         []() { return true; },
//         []() { 
//             intake_top.move_voltage(6000); 
//             intake_bottom.move_voltage(12000);
//             Miku.move_voltage(4000, 4000);
//         }
//     });
//     barrier_cross->add_event(ConditionalEvent{
//         [barrier_cross]() { return barrier_cross->events[0].triggered
//             && floor_optical.get_color(RED); },
//         []() { }
//     });
//     barrier_cross->add_event(ConditionalEvent{
//         [barrier_cross]() { return barrier_cross->events[1].triggered
//             && floor_optical.get_color(TILE); },
//         []() { }
//     });
//     barrier_cross->add_event(ConditionalEvent{
//         [barrier_cross]() { return barrier_cross->events[2].triggered
//             && floor_optical.get_color(RED); },
//         []() { }
//     });
//     barrier_cross->add_event(ConditionalEvent{
//         [barrier_cross]() { return barrier_cross->events[3].triggered
//             && floor_optical.get_color(TILE); },
//         [barrier_cross]() { barrier_cross->done = true; }
//     });
//     queue_motion(barrier_cross);


// }

// 80 skills

void skills() {
    queue_motion(new MovePoint({47, -48}, 1500, {.reverse = true}));
    auto turn_loader_1 = new TurnHeading(180, 1000);
    turn_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); descore_piston.set_value(true); intake.set(4000, 12000); }
    });
    queue_motion(turn_loader_1);
    queue_motion(new MovePoint({47, -64}, 1500, {.drive_max_volt_pct = 50}));
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    auto move_loader_1 = new MovePoint({62, -30}, 1500, {.reverse = true, .cutoff = 2.0});
    move_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(3000, 0); intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); }
    });
    queue_motion(move_loader_1);
    queue_motion(new TurnHeading(180, 1000, {.cutoff = 5.0}));
    queue_motion(new MovePoint({62, 30}, 1500, {.reverse = true, .cutoff = 2.0}));
    queue_motion(new MovePoint({48, 48}, 1500, {.reverse = true, .drive_max_volt_pct = 40}));
    queue_motion(new TurnHeading(0, 1000));
    auto score_loader_1 = new MovePoint({48, 20}, 1000, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); }
    });
    score_loader_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set_anti_jam(true); intake.set(12000, 12000); }
    });
    queue_motion(score_loader_1);
    queue_motion(new Delay(1500));
    queue_motion(new TurnHeading(0, 1000));
    auto move_loader_2 = new MovePoint({47, 64}, 1000, {.drive_max_volt_pct = 50});
    move_loader_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake.set_anti_jam(false); intake.set(4000, 12000); }
    });
    queue_motion(move_loader_2);
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    auto score_loader_2 = new MovePoint({48, 20}, 1000, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set_anti_jam(true); intake.set(12000, 12000); loader_piston.set_value(false); }
    });
    queue_motion(score_loader_2);
    queue_motion(new Delay(1500));
    auto turn_from_score_2 = new SwingHeading(-100, 1000, {.locked_side = Side::LEFT});
    turn_from_score_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); loader_piston.set_value(false); intake.set_anti_jam(false); intake.set(4000, 12000); }
    });
    queue_motion(turn_from_score_2);
    auto block_group_1 = new MovePoint({16, 16}, 2000, {.drive_max_volt_pct = 60});
    block_group_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({16, 16}) < 12.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_1);
    queue_motion(new TurnHeading(45, 1000));
    auto mid_score_1 = new MovePoint({8, 8}, 1500, {.reverse = true});
    mid_score_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { middle_piston.set_value(true); intake.set(-12000, -3000);}
    });
    mid_score_1->add_event(ConditionalEvent{
        [mid_score_1]() { return pros::millis() - mid_score_1->start_time > 200; },
        []() { intake.set({400, VELOCITY}, {12000, VOLTAGE}); }
    });
    queue_motion(mid_score_1);
    queue_motion(new TurnHeading(45, 1000));
    queue_motion(new Delay(1000));
    queue_motion(new MovePoint({20, 20}, 1500));
    auto turn_block_group_3 = new TurnHeading(-90, 1000);
    turn_block_group_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(4000, 12000); loader_piston.set_value(false); middle_piston.set_value(false); }
    });
    queue_motion(turn_block_group_3);
    auto block_group_3 = new MovePoint({-24, 24}, 2000, {.drive_max_volt_pct = 60});
    block_group_3->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-24, 24}) < 12.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_3);
    queue_motion(new Delay(500));
    queue_motion(new TurnHeading(135, 1000));
    queue_motion(new MovePoint({-48, 40}, 3000, {.reverse = true}));
    queue_motion(new TurnHeading(0, 1000));
    auto score_loader_2b = new MovePoint({-48, 20}, 1000, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_2b->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set_anti_jam(true); intake.set(12000, 12000);}
    });
    queue_motion(score_loader_2b);
    queue_motion(new Delay(1000));
    queue_motion(new TurnHeading(0, 1000));

    auto move_loader_3 = new MovePoint({-47, 64}, 1500, {.drive_max_volt_pct = 50});
    move_loader_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake.set_anti_jam(false); intake.set(4000, 12000); }
    });
    queue_motion(move_loader_3);
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    auto pre_score_loader_3 = new MovePoint({-62, 30}, 2000, {.reverse = true, .cutoff = 2.0});
    pre_score_loader_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set(3000, 0); intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); }
    });
    queue_motion(pre_score_loader_3);
    queue_motion(new TurnHeading(0, 1000, {.cutoff = 5.0}));
    queue_motion(new MovePoint({-62, -30}, 2000, {.reverse = true, .cutoff = 2.0}));
    queue_motion(new MovePoint({-48, -48}, 1500, {.reverse = true, .drive_max_volt_pct = 40}));
    queue_motion(new TurnHeading(180, 1000));
    auto score_loader_3 = new MovePoint({-47, -20}, 1000, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); }
    });
    score_loader_3->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-47, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set_anti_jam(true); intake.set(12000, 12000); }
    });
    queue_motion(score_loader_3);
    queue_motion(new Delay(1500));
    queue_motion(new TurnHeading(180, 1000));
    auto move_loader_4 = new MovePoint({-47, -64}, 1500, {.drive_max_volt_pct = 50});
    move_loader_4->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake.set_anti_jam(false); intake.set(4000, 12000); }
    });
    queue_motion(move_loader_4);
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    queue_motion(new MoveTime(-4000, -4000, 50));
    queue_motion(new MoveTime(4000, 4000, 500));
    auto score_loader_4 = new MovePoint({-47, -20}, 1000, {.reverse = true, .drive_max_volt_pct = 60});
    score_loader_4->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-47, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake.set_anti_jam(true); intake.set(12000, 12000); loader_piston.set_value(false); }
    });
    queue_motion(score_loader_4);
    queue_motion(new Delay(1500));
    queue_motion(new MovePoint({-18, -64}, 1500));
    auto park_clear_align = new TurnHeading(90, 500);
    park_clear_align->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(park_clear_align);
    auto park_clear = new MoveTime(8000, 8000, 1000);
    queue_motion(park_clear);

}