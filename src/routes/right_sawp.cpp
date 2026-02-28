#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void sawp() {

    wait(100)
        .start([]() { intake.load(); })
        .queue();
    move_time(12000, 4000, 200).queue();
    turn_point({47, -48}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({47, -48}, 1200, {.reverse = true, .quick_exit = false, .drive_max_volt_pct = 70}).queue();
    turn_point({47, -60}, 750)
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_pose({47, -60}, 180, 1000).queue();
    // turn_point({47, -72}, 300, {.cutoff = 5.0}).queue();
    move_time(6000, 6000, 500).queue();
    move_pose({48, -24}, 180, 1000, {.reverse = true})
        .within(8.0, []() { intake.score(); })
        .queue();
    move_time(-2000, -2000, 700)
        .start([]() { intake.score(); })
        .queue();
    swing_point({24, -24}, 750, {.locked_side = Side::RIGHT, .cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { loader_piston.set_value(false); intake.load(); })
        .queue();
    move_point({24, -24}, 1000, {.min_volt_pct = 20}).queue();
    move_point({-24, -24}, 1500, {.drive_max_volt_pct = 60, .min_volt_pct = 20})
        .event(ConditionalEvent{
            []() { return Miku.get_position().x < -12; },
            []() { loader_piston.set_value(true); }
        })
        .queue();
    turn_point({-48, -42}, 500, {.cutoff = 10.0}).queue();
    move_pose({-48, -42}, -90, 1000).queue();
    turn_point({-48, -24}, 500, {.reverse = true}).queue();
    move_pose({-48, -24}, 180, 1000, {.reverse = true})
        .within(8.0, []() { intake.score(); })
        .queue();
    move_time(-2000, -2000, 1000)
        .start([]() { intake.score(); })
        .queue();
    move_pose({-47, -60}, 180, 1000)
        .start([]() { intake.load(); })
        .queue();
    move_time(6000, 6000, 500).queue();
    move_pose({-8, -12}, -135, 2000, {.reverse = true, .max_vel_pct = 60})
        .within(4.0, []() { intake.queue_spin(-12000, 100); intake.set_top_velocity(-150); intake.set_middle_velocity(150); intake.set_bottom(12000); loader_piston.set_value(false); })
        .queue();
    wait(500)
        .start([]() { intake.set(-8000, 12000); })
        .queue();
    
}