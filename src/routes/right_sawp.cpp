#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void sawp() {

    wait(100)
        .start([]() { intake.load(); })
        .queue();
    move_time(12000, 6000, 300).queue();
    turn_point({47, -48}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({47, -48}, 1000, {.reverse = true, .quick_exit = false, .drive_max_volt_pct = 75}).queue();
    turn_point({47, -60}, 500, {.cutoff = 10.0})
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_pose({47, -60}, 180, 1000, {.max_vel_pct = 20}).queue();
    move_time(4000, 4000, 500).queue();
    move_pose({48, -24}, 180, 1000, {.reverse = true})
        .within(6.0, []() { intake.score(); })
        .queue();
    move_time(-2000, -2000, 500)
        .start([]() { intake.score(); })
        .queue();
    swing_point({24, -24}, 1000, {.locked_side = Side::RIGHT})
        .start([]() { loader_piston.set_value(false); intake.load(); })
        .queue();
    move_point({24, -24}, 1000, {.min_volt_pct = 20}).queue();
    move_point({-24, -24}, 1500, {.drive_max_volt_pct = 50, .min_volt_pct = 20})
        .event(ConditionalEvent{
            []() { return Miku.get_position().x < -12; },
            []() { loader_piston.set_value(true); }
        })
        .queue();
    turn_point({-48, -42}, 500, {.cutoff = 10.0}).queue();
    move_pose({-48, -42}, -90, 1000).queue();
    turn_point({-48, -24}, 500, {.reverse = true}).queue();
    move_pose({-48, -24}, 180, 1000, {.reverse = true})
        .within(6.0, []() { intake.score(); })
        .queue();
    move_time(-2000, -2000, 1000)
        .start([]() { intake.score(); })
        .queue();
    move_pose({-47, -60}, 180, 1000)
        .start([]() { intake.load(); })
        .queue();
    move_time(4000, 4000, 500).queue();
    move_pose({-10, -10}, -135, 2000, {.reverse = true, .max_vel_pct = 60})
        .within(3.0, []() { intake.queue_spin(-12000, 100); intake.set(-8000, 12000); loader_piston.set_value(false); })
        .queue();
    wait(500)
        .start([]() { intake.set(-8000, 12000); })
        .queue();
    
}