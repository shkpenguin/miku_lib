#include "macro.hpp"
#include "miku/miku-api.hpp"

void descore_align() {
    // Miku.set_position({0, 0});
    // if(Miku.get_heading().degrees().norm() > 180) {
    //     queue_motion(new MovePoint({0, -8}, 300, {.cutoff = 1.0, .min_volt_pct = 50}));
    //     queue_motion(new TurnHeading(-135, 300, {.cutoff = 5.0, .min_volt_pct = 20}));
    //     queue_motion(new MoveTime(-12000, -5000, 500));
    //     queue_motion(new MoveTime(-4000, -12000, 200));
    // } else {
    //     queue_motion(new MovePoint({0, 24}, 1000, {.drive_max_volt_pct = 80}));
    // }
}

void park_clear() {
    move_time(4000, 4000, 1000)
        .start([]() { intake.load(); })
        .end([]() { return floor_optical.get_color(RED); })
        .queue();
    move_time(2000, 2000, 500).queue();
    wait(500).queue();
    move_time(4000, 4000, 1000).queue();
    move_time(-4000, -4000, 200).queue();
    move_time(4000, 4000, 500).queue();
    move_time(-4000, -4000, 1500).queue();
}