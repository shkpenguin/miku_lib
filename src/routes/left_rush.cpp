#include "miku/miku-api.hpp"
#include "miku/motions.hpp"



void left_rush() {

    move_point({-24, -24}, 800, {.drive_max_volt_pct = 50})
        .start([] { intake.set(12000, 12000); descore_piston.set_value(true); })
        .elapsed(500, []() { loader_piston.set_value(true); })
        .queue();
    turn_point({-47, -42}, 400, {.cutoff = 10.0})
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_pose({-48, -42}, -90, 900)
        .elapsed(700, []() { intake.stop(); })
        .queue();

    turn_point({-48, -24}, 400, {.reverse = true})
        .start([]() { intake.stop(); lock_piston.set_value(false); })
        .queue();
    move_point({-48, -24}, 500, {.reverse = true})
        .queue();
    move_time(-3000, -3000, 200)
        .start([]() { intake.score(); })
        .queue();
    wait(1200).queue();

    move_point({-48, -40}, 500)
        .event({
            [] { return true; },
            [] { lock_piston.set_value(true); loader_piston.set_value(false); }
        })
        .queue();

    turn_point({-38, -30}, 300, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({-38, -30}, 500, {.reverse = true, .cutoff = 2.0}).queue();
    turn_point({-39, -12}, 300, {.reverse = true})
        .event(start([]() { descore_piston.set_value(false); }))
        .queue();
    move_point({-39, -12}, 200, {.reverse = true, .cutoff = 5.0, .min_volt_pct = 75}).queue();
    move_time(-6000, -6000, 400)
        .end([]() { return Miku.get_pose().y > -12.0; })
        .queue();
    move_time(6000, 6000, 50).queue();

}