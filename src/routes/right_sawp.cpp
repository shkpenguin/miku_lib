#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> sawp_paths = {};

void sawp() {
    move_time(6000, 6000, 200)
        .event(start([]() { descore_piston.set_value(true); intake.set(4000, 12000); }))
        .queue();
    wait(100).queue();
    move_point({47, -48}, 1200, {.reverse = true, .drive_max_volt_pct = 75}).queue();
    turn_point({47, -72}, 600, {.cutoff = 5.0})
        .event(start([]() { loader_piston.set_value(true); }))
        .queue();
    move_time(6000, 6000, 500).queue();
    wait(200).queue();
    
    move_pose({48, -24}, 180, 1500, {.reverse = true})
        .event(within({48, -24}, 6.0, []() { intake.set(12000, 12000); lock_piston.set_value(true); }))
        .queue();
    wait(500).queue();
    
    swing_heading(-80, 600, {.locked_side = Side::RIGHT, .cutoff = 5.0})
        .event(start([]() { loader_piston.set_value(false); }))
        .queue();
    
    move_point({24, -24}, 2000, {.drive_max_volt_pct = 50})
        .events({
            start([]() { intake.set(4000, 12000); lock_piston.set_value(false); }),
            within({24, -24}, 16.0, []() { loader_piston.set_value(true); })
        })
        .queue();
    
    move_point({-24, -24}, 2000, {.drive_max_volt_pct = 50})
        .events({
            start([]() { loader_piston.set_value(false); }),
            within({-24, -24}, 16.0, []() { loader_piston.set_value(true); })
        })
        .queue();
    
    turn_heading(-135, 300).queue();
    
    move_pose({-8, -8}, -135, 1000, {.reverse = true})
        .event(start([]() { middle_piston.set_value(true); }))
        .event(elapsed(200, []() { intake.set(-12000, -6000); }))
        .event(elapsed(600, []() { intake.set_top({400, VELOCITY}); intake.set_bottom({12000, VOLTAGE}); }))
        .queue();
    
    turn_heading(-135, 300).queue();
    wait(300).queue();
    
    move_point({-47, -48}, 1000, {.drive_max_volt_pct = 75})
        .event(start([]() { intake.set(12000, 0); }))
        .queue();
    
    turn_point({-47, -72}, 400, {.cutoff = 5.0})
        .event(start([]() { middle_piston.set_value(false); intake.set(4000, 12000); }))
        .queue();
    
    move_time(6000, 6000, 500).queue();
    wait(200).queue();
    
    move_pose({-48, -24}, 180, 1500, {.reverse = true})
        .event(within({-48, -24}, 6.0, []() { intake.set(12000, 12000); lock_piston.set_value(true); }))
        .queue();
}