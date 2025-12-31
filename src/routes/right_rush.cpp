#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void right_rush() {
    move_point({24, -24}, 1500)
        .events({
            {[] { return true; }, [] { intake.set(4000, 12000); }},
            {[] { return Miku.get_pose().distance_to({24, -24}) < 12.0; }, [] { loader_piston.set_value(true); }}
        })
        .queue();
    pros::delay(200);
    turn_heading(135, 400, {.cutoff = 5.0}).queue();
    move_point({47, -48}, 1200).queue();
    turn_point({47, -72}, 500)
        .start([]() { loader_piston.set_value(true); intake.load(); })
        .queue();
    move_pose({47, -54}, 180, 1000, {.max_vel_pct = 30})
        .queue();
    turn_heading(180, 300).queue();
    move_time(4000, 4000, 500).queue();
    wait(500).queue();

    move_pose({48, -24}, 180, 1500, {.reverse = true})
        .event(within({48, -24}, 8.0, []() { intake.set(12000, 12000); lock_piston.set_value(true); }))
        .queue();
    wait(1000)
        .event(start([]() { intake.set(12000, 12000); lock_piston.set_value(true); }))
        .queue();

    // illegal wing push (now legal)
    move_point({48, -40}, 500)
        .event({
            [] { return true; },
            [] { lock_piston.set_value(false); loader_piston.set_value(false); }
        })
        .queue();

    turn_point({58, -30}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({58, -30}, 700, {.reverse = true, .cutoff = 2.0}).queue();
    turn_point({57, -12}, 300, {.reverse = true})
        .event(start([]() { descore_piston.set_value(false); }))
        .queue();
    move_point({57, -12}, 200, {.reverse = true, .cutoff = 5.0, .min_volt_pct = 75}).queue();
    move_time(-6000, -6000, 500)
        .end([]() { return Miku.get_pose().y > -10.0; })
        .queue();

}