#include "miku/miku-api.hpp"
#include "miku/motions.hpp"



void left_rush() {

    move_point({-47, -48}, 1500, {.drive_max_volt_pct = 75})
        .start([]() { intake.stop(); })
        .queue();
    turn_point({-47, -72}, 500)
        .start([]() { loader_piston.set_value(true); intake.load(); })
        .queue();
    move_pose({-47, -54}, 180, 1000, {.max_vel_pct = 30})
        .queue();
    turn_heading(180, 300).queue();
    move_time(4000, 4000, 500).queue();
    wait(300).queue();

    move_pose({-48, -24}, 180, 1000, {.reverse = true})
        .event(within({-48, -24}, 6.0, []() { intake.score(); }))
        .queue();
    wait(300).queue();
    
    swing_point({-24, -24}, 1000, {.locked_side = Side::LEFT, .cutoff = 5.0})
        .event(start([]() { loader_piston.set_value(false); }))
        .queue();

    move_point({-24, -24}, 2000, {.cutoff = 3.0, .drive_max_volt_pct = 75, .min_volt_pct = 20})
        .events({
            start([]() { intake.load(); }),
            within({-24, -24}, 16.0, []() { loader_piston.set_value(true); })
        })
        .queue();
    
    turn_point({-44, -6}, 500)
        .event(start([]() { loader_piston.set_value(false); }))
        .queue();
    move_point({-44, -6}, 1500, {.drive_max_volt_pct = 60})
        .within({-42, -8}, 6.0, []() { loader_piston.set_value(true); })
        .queue();
    
    wait(200).queue();

    move_point({-24, -24}, 1000, {.reverse = true, .drive_max_volt_pct = 60}).queue();
    turn_point({-7, -7}, 500, {.reverse = true}).queue();
    move_pose({-7, -7}, -135, 1500, {.reverse = true, .max_vel_pct = 30})
        .event(start([]() { intake.set(-6000, -2000); }))
        .event(elapsed(200, []() { intake.stop(); middle_piston.set_value(true); }))
        .within({-7, -7}, 3.0, []() { intake.set_top_velocity(150); intake.set_bottom(8000); })
        .queue();
    turn_point({0, 0}, 300, {.reverse = true})
        .start([]() { intake.set_top_velocity(150); intake.set_bottom(8000); })
        .queue();
    wait(1000)
        .event(start([]() { loader_piston.set_value(false); intake.set_top_velocity(200); intake.set_bottom(12000); }))
        .queue();
    // */

}   