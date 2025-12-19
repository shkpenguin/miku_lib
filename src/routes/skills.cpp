#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {};

void shimmy() {
    move_time(-4000, -4000, 100).queue();
    move_time(4000, 4000, 200).queue();
    wait(100).queue();
    move_time(-4000, -4000, 100).queue();
    move_time(4000, 4000, 200).queue();
    wait(100).queue();
    move_time(-4000, -4000, 100).queue();
    move_time(4000, 4000, 200).queue();
    wait(100).queue();
    move_time(-4000, -4000, 100).queue();
    move_time(4000, 4000, 200).queue();
    wait(100).queue();
}

void skills() {
    move_point({24, -24}, 1500, {.drive_max_volt_pct = 50})
        .events({
            start([] { intake.set(4000, 12000); }),
            within({24, -24}, 15.0, [] { loader_piston.set_value(true); })
        })
        .queue();
    turn_point({48, -40}, 400, {.cutoff = 5.0}).queue();
    move_point({48, -40}, 800, {.cutoff = 2.0, .min_volt_pct = 20}).queue();
    turn_point({48, -24}, 500, {.reverse = true, .cutoff = 5.0})
        .queue();
    move_pose({48, -24}, 180, 1000, {.reverse = true})
        .event(within({48, -24}, 6.0, [] { lock_piston.set_value(true); intake.set(12000, 12000); }))
        .queue();
    wait(500).queue();
    move_pose({48, -48}, 180, 1000)
        .event(start([] { lock_piston.set_value(false); }))
        .queue();
    move_time({6000, 6000, 500}).queue();
    shimmy();
    move_pose({60, -30}, 180, 1500, {.reverse = true, .cutoff = 5.0, .min_vel_pct = 30}).queue();
    move_pose({60, 30}, 180, 1000, {.reverse = true, .cutoff = 2.0, .min_vel_pct = 30}).queue();
    move_point({48, 48}, 1500, {.reverse = true, .cutoff = 5.0, .drive_max_volt_pct = 50}).queue();
    turn_point({48, 24}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_pose({48, 24}, 0, 1000, {.reverse = true, .max_vel_pct = 30})
        .event(within({48, 24}, 6.0, [] { lock_piston.set_value(true); intake.set(12000, 12000); loader_piston.set_value(false); }))
        .queue();
    wait(500).queue();
    move_pose({18, 65}, -90, 2000).queue();
    move_time(8000, 8000, 1500)
        .seq({
            start([]() { loader_piston.set_value(true); }),
            ConditionalEvent(
                []() { return floor_optical.get_color(BLUE); },
                []() { loader_piston.set_value(true); }),
            await([]() { return floor_optical.get_color(TILE); }),
            await([]() { return floor_optical.get_color(BLUE); }),
            await([]() { return floor_optical.get_color(TILE); })
        })
        .queue();
    move_time(-4000, -4000, 500).queue();
    move_pose({36, 62}, -90, 2000).queue();

}