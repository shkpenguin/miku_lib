#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void right_rush() {
    move_point({24, -24}, 1500)
        .events({
            {[] { return true; }, [] { intake.set(4000, 12000); }},
            {[] { return Miku.get_pose().distance_to({24, -24}) < 12.0; }, [] { loader_piston.set_value(true); }}
        })
        .queue();
    turn_point({47, -48}, 500, {.cutoff = 5.0}).queue();
    move_point({47, -42}, 1200).queue();
    turn_point({47, -54}, 500, {.cutoff = 5.0}).queue();
    move_pose({47, -54}, 180, 1000).queue();
    turn_heading(180, 300, {.cutoff = 5.0}).queue();
    move_time(6000, 6000, 500).queue();
    wait(300).queue();

    move_pose({48, -24}, 180, 1500, {.reverse = true, .max_vel_pct = 50})
        .event(within({48, -29}, 3.0, []() { intake.score(); }))
        .queue();
    wait(1000)
        .event(start([]() { intake.score(); }))
        .queue();

    // move_point({48, -40}, 500)
    //     .event({
    //         [] { return true; },
    //         [] { lock_piston.set_value(false); loader_piston.set_value(false); }
    //     })
    //     .queue();

    // turn_point({58, -30}, 500, {.reverse = true, .cutoff = 5.0}).queue();

    // move_time(6000, 6000, 200).queue();
    // move_point({58, -30}, 700, {.cutoff = 2.0}).queue();
    // turn_point({57, -12}, 300, {.reverse = true})
    //     .event(start([]() { descore_piston.set_value(false); }))
    //     .queue();

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
    move_time(-6000, -6000, 400)
        .end([]() { return Miku.get_pose().y > -12.0; })
        .queue();
    move_time(6000, 6000, 50).queue();

}