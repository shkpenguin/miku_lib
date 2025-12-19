#include "miku/miku-api.h"
#include "miku/motions.h"

std::vector<std::reference_wrapper<BezierPath>> rush_paths = {};

void right_rush() {
    move_point({24, -24}, 1500)
        .events({
            {[] { return true; }, [] { intake.set(4000, 12000); }},
            {[] { return Miku.get_pose().distance_to({24, -24}) < 12.0; }, [] { loader_piston.set_value(true); }}
        })
        .queue();

    turn_heading(135, 400, {.cutoff = 5.0}).queue();
    move_point({48, -48}, 800, {.cutoff = 3.0}).queue();
    turn_heading(180, 300, {.cutoff = 5.0}).queue();
    move_point({47, -64}, 1000).queue();
    move_time(3000, 3000, 100).queue();

    move_point({48, -20}, 1500, {.reverse = true, .drive_max_volt_pct = 50})
        .event({
            [] { return Miku.get_pose().distance_to(Point(48, -24)) < 5.0; },
            [] { lock_piston.set_value(true); intake.set(12000, 12000); }
        })
        .queue();

    wait(500).queue();

    // illegal wing push (now legal)
    move_point({48, -40}, 500)
        .event({
            [] { return true; },
            [] { lock_piston.set_value(false); loader_piston.set_value(false); }
        })
        .queue();

    turn_heading(-135, 300).queue();
    move_point({56, -30}, 700, {.reverse = true}).queue();
    turn_heading(180, 300).queue();
    move_point({57, -10}, 1500, {.reverse = true, .cutoff = 5.0, .min_volt_pct = 75}).queue();

    move_time(0, 0, 500)
        .event({[] { return true; }, [] { Miku.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); }})
        .queue();
}