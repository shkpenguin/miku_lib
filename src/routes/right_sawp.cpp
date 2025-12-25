#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

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
    wait(500).queue();

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
    
    move_point({-24, -24}, 2000, {.drive_max_volt_pct = 75})
        .events({
            start([]() { loader_piston.set_value(false); }),
            within({-24, -24}, 16.0, []() { loader_piston.set_value(true); })
        })
        .queue();
    
    move_point({-48, -48}, 1000, {.drive_max_volt_pct = 75, .cos_scale = 2.0}).queue();

    turn_point({-48, -24}, 500, {.reverse = true}).queue();

    move_pose({-48, -24}, 180, 1000, {.reverse = true})
        .event(within({-48, -24}, 6.0, []() { intake.set(12000, 12000); lock_piston.set_value(true); }))
        .queue();
    wait(500).queue();

    move_point({-47, -48}, 1000, {.drive_max_volt_pct = 50})
        .event(start([]() { lock_piston.set_value(false); }))
        .queue();
    turn_point({-47, -72}, 300);
    move_time(6000, 6000, 500).queue();
    wait(500).queue();
    
    move_time(-4000, -4000, 300).queue();
    move_point({-24, -24}, 1500, {.reverse = true, .cutoff = 3.0, .drive_max_volt_pct = 50}).queue();
    move_pose({-8, -8}, -135, 2000, {.reverse = true, .max_vel_pct = 30})
        .event(start([]() { intake.set(-12000, -6000); }))
        .event(elapsed(200, []() { intake.set(0, 0); middle_piston.set_value(true); }))
        .event(elapsed(800, []() { intake.set_top_velocity(200); intake.set_bottom(12000); }))
        .queue();

    turn_point({0, 0}, 300);
    
}