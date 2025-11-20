#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {};

void skills() {
    queue_motion(new MovePoint({48, -48}, 1500, {.reverse = true}));
    auto turn_loader_1 = new TurnHeading(180, 1000);
    turn_loader_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); descore_piston.set_value(true); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(turn_loader_1);
    queue_motion(new MovePoint({48, -67}, 1500));
    queue_motion(new MovePoint({48, -67}, 1500));
    queue_motion(new Delay(1000));
    queue_motion(new MovePoint({64, -24}, 1500, {.reverse = true}));
    queue_motion(new TurnHeading(180, 1000));
    queue_motion(new MovePoint({64, 24}, 1500, {.reverse = true}));
    queue_motion(new MovePoint({48, 48}, 1500, {.reverse = true}));
    queue_motion(new TurnHeading(0, 1000));
    auto score_loader_1 = new MovePoint({48, 20}, 1000, {.reverse = true, .max_speed = 6000});
    score_loader_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); }
    });
    queue_motion(score_loader_1);
    queue_motion(new Delay(1000));
    queue_motion(new TurnHeading(0, 1000));
    auto move_loader_2 = new MovePoint({48, 67}, 1000, {.max_speed = 6000});
    move_loader_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(move_loader_2);
    queue_motion(new Delay(1000));
    auto score_loader_2 = new MovePoint({48, 20}, 1000, {.reverse = true, .max_speed = 6000});
    score_loader_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); loader_piston.set_value(false); }
    });
    queue_motion(score_loader_2);
    queue_motion(new Delay(1000));
    queue_motion(new MovePoint({48, 48}, 3000));
    queue_motion(new TurnPoint({24, 24}, 1000));
    auto block_group_1 = new MovePoint({24, 24}, 1000, {.max_speed = 6000});
    block_group_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    block_group_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, 24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_1);
    auto block_group_2 = new MovePoint({24, -24}, 2000, {.max_speed = 6000});
    block_group_2->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(false); }
    });
    block_group_2->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({24, -24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_2);
    auto pre_mid_score_1 = new MovePoint({24, 24}, 1500, {.reverse = true, .max_speed = 6000});
    pre_mid_score_1->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(pre_mid_score_1);
    queue_motion(new TurnHeading(45, 1000));
    auto mid_score_1 = new MovePoint({10, 10}, 1500, {.reverse = true});
    mid_score_1->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(10, 10)) < 3.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(10000); intake_bottom.move_voltage(10000); loader_piston.set_value(false); }
    });
    queue_motion(mid_score_1);
    queue_motion(new TurnHeading(45, 1000));
    queue_motion(new Delay(2000));
    queue_motion(new MovePoint({24, 24}, 1500));
    queue_motion(new TurnHeading(-90, 1000));
    auto block_group_3 = new MovePoint({-24, 24}, 1000);
    block_group_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    block_group_3->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to({-30, 24}) < 16.0; },
        []() { loader_piston.set_value(true); }
    });
    queue_motion(block_group_3);
    queue_motion(new Delay(500));
    queue_motion(new TurnHeading(90, 1000));
    queue_motion(new MovePoint({-48, 36}, 3000, {.reverse = true}));
    queue_motion(new TurnHeading(0, 1000));
    auto score_loader_2b = new MovePoint({-48, 20}, 1000, {.reverse = true, .max_speed = 6000});
    score_loader_2b->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-48, 24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); }
    });
    queue_motion(score_loader_2b);
    queue_motion(new Delay(1000));
    queue_motion(new TurnHeading(0, 1000));

    auto move_loader_3 = new MovePoint({-48, 67}, 1500, {.max_speed = 6000});
    move_loader_3->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(move_loader_3);
    queue_motion(new Delay(1000));
    queue_motion(new MovePoint({-64, 24}, 2000, {.reverse = true}));
    queue_motion(new TurnHeading(0, 1000));
    queue_motion(new MovePoint({-64, -24}, 2000, {.reverse = true}));
    queue_motion(new MovePoint({-48, -48}, 1500, {.reverse = true}));
    queue_motion(new TurnHeading(180, 1000));
    auto score_loader_3 = new MovePoint({-48, -20}, 1000, {.reverse = true, .max_speed = 6000});
    score_loader_3->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-48, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); }
    });
    queue_motion(score_loader_3);
    queue_motion(new Delay(1000));
    queue_motion(new TurnHeading(180, 1000));
    auto move_loader_4 = new MovePoint({-48, -67}, 1500, {.max_speed = 6000});
    move_loader_4->add_event(ConditionalEvent{
        []() { return true; },
        []() { lock_piston.set_value(false); intake_top.move_voltage(4000); intake_bottom.move_voltage(12000); }
    });
    queue_motion(move_loader_4);
    queue_motion(new Delay(1000));
    auto score_loader_4 = new MovePoint({-47, -20}, 1000, {.reverse = true, .max_speed = 6000});
    score_loader_4->add_event(ConditionalEvent{
        []() { return Miku.get_pose().distance_to(Point(-48, -24)) < 5.0; },
        []() { lock_piston.set_value(true); intake_top.move_voltage(12000); loader_piston.set_value(false); }
    });
    queue_motion(score_loader_4);
    queue_motion(new Delay(1000));
    queue_motion(new MovePoint({-18, -64}, 1500));
    auto park_align = new TurnHeading(90, 1000);
    park_align->add_event(ConditionalEvent{
        []() { return true; },
        []() { loader_piston.set_value(true); }
    });
    auto park = new MoveTime(8000, 8000, 1000);
    
    queue_motion(park);

}