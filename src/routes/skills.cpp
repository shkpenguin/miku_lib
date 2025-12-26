#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void skills() {
    auto shimmy = []() {
        wait(500).queue();
        move_time(-4000, -4000, 50).queue();
        move_time(4000, 4000, 200).queue();
        wait(100).queue();
        move_time(-4000, -4000, 50).queue();
        move_time(4000, 4000, 200).queue();
        wait(100).queue();
        move_time(-4000, -4000, 50).queue();
        move_time(4000, 4000, 200).queue();
        wait(300).queue();
    };
    
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
    move_pose({48, -24}, 180, 1200, {.reverse = true})
        .event(within({48, -24}, 6.0, [] { intake.score(); }))
        .queue();
    wait(500)
        .start([]() { intake.score(); })
        .queue();
    turn_point({47, -48}, 300).queue();
    move_point({47, -48}, 500, {.cutoff = 3.0, .drive_max_volt_pct = 50}).queue();
    move_pose({47, -54}, 180, 1000, {.max_vel_pct = 30})
        .event(start([] { intake.load(); }))
        .queue();
    turn_heading(180, 300).queue();
    move_time({4000, 4000, 500}).queue();
    shimmy();
    move_pose({60, -30}, 180, 1500, {.reverse = true, .cutoff = 5.0, .min_vel_pct = 30}).queue();
    move_point({60, 32}, 1000, {.reverse = true, .cutoff = 2.0, .drive_max_volt_pct = 75, .min_volt_pct = 20}).queue();
    move_point({48, 40}, 1500, {.reverse = true, .cutoff = 5.0, .drive_max_volt_pct = 40}).queue();
    turn_point({48, 20}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({48, 20}, 1500, {.reverse = true})
        .event(within({48, 24}, 6.0, [] { intake.score(); }))
        .queue();
    wait(1000)
        .start([]() { intake.score(); })
        .queue();
    turn_point({47, 48}, 300).queue();
    move_point({47, 48}, 500, {.cutoff = 3.0, .drive_max_volt_pct = 50}).queue();
    move_pose({47, 54}, 0, 1000, {.max_vel_pct = 30})
        .event(start([]() { intake.load(); }))
        .queue();
    turn_heading(0, 300).queue();
    move_time(4000, 4000, 500).queue();
    shimmy();
    move_pose({48, 24}, 0, 1500, {.reverse = true, .max_vel_pct = 40})
        .event(within({48, 24}, 6.0, [] { intake.score(); }))
        .queue();
    wait(1300)
        .event(start([]() { loader_piston.set_value(false); intake.score(); }))
        .queue();
    move_pose({18, 63}, -90, 2000, {.max_vel_pct = 35})
        .event(start([]() { intake.load(); }))
        .queue();
    turn_point({-18, 65}, 500).queue();
    wait(300)
        .event(start([]() { loader_piston.set_value(true); }))
        .queue();
    move_time(8000, 7000, 2000)
        .seq({
            elapsed(200, []() { loader_piston.set_value(false); }),
            await([]() { return floor_optical.get_color(BLUE); }),
            ConditionalEvent(
                []() { return floor_optical.get_color(TILE); },
                []() { Miku.move_voltage(6000, 6000); }),
            await([]() { return floor_optical.get_color(BLUE); })
        })
        .end_seq([]() { return floor_optical.get_color(TILE); })
        .queue();
    move_point({-44, 65}, 2000, {.drive_max_volt_pct = 30})
        .event(start([]() { Miku.distance_reset({-18, 65}); }))
        .queue();
    move_point({-24, 48}, 1000, {.reverse = true, .drive_max_volt_pct = 50}).queue();
    move_point({-48, 48}, 1500, {.drive_max_volt_pct = 50}).queue();
    turn_point({-48, 24}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_pose({-48, 24}, 0, 1000, {.reverse = true, .max_vel_pct = 40})
        .event(within({-48, 24}, 8.0, [] { intake.score(); }))
        .queue();
    wait(1000)
        .start([]() { intake.score(); })    
        .queue();
    turn_point({-47, 48}, 300)
        .event(start([]() { loader_piston.set_value(true); }))
        .queue();
    move_point({-47, 48}, 500, {.cutoff = 3.0, .drive_max_volt_pct = 50}).queue();
    move_pose({-47, 54}, 0, 1000, {.max_vel_pct = 30})
        .event(start([]() { intake.load(); }))
        .queue();
    turn_heading(0, 300).queue();
    move_time(4000, 4000, 500).queue();
    shimmy();
    move_pose({-48, 24}, 0, 1200, {.reverse = true, .max_vel_pct = 40})
        .event(within({-48, 24}, 8.0, [] { intake.score(); }))
        .queue();
    wait(1000)
        .event(start([]() { loader_piston.set_value(false); intake.score(); }))
        .queue();
    swing_point({-24, 24}, 1000, {.cutoff = 5.0})
        .start([]() { intake.load(); })
        .queue();
    move_pose({-24, 24}, 135, 1000, {.max_vel_pct = 30, .min_vel_pct = 15})
        .queue();
    wait(500).queue();
    move_pose({-8, 8}, 135, 1000, {.max_vel_pct = 30})
        .queue();
    turn_point({0, 0}, 300)
        .event(start([]() { intake.set_top(0), intake.set_bottom_velocity(-300); }))
        .queue();
    move_time(-3000, -3000, 100).queue();
    wait(200).queue();
    move_time(3000, 3000, 100).queue();
    wait(200).queue();
    move_time(-3000, -3000, 100)
        .start([]() { intake.set_top(-4000); })
        .queue();
    wait(200).queue();
    move_time(3000, 3000, 100).queue();
    wait(500).queue();
    move_point({-20, 20}, 800, {.reverse = true, .cutoff = 2.0}).queue();
    turn_point({-24, -24}, 500, {.cutoff = 5.0}).queue();
    move_point({-24, -24}, 1500, {.drive_max_volt_pct = 60})
        .event(start([] { intake.set(4000, 12000); }))
        .event(within({-24, -24}, 12.0, [] { loader_piston.set_value(true); }))
        .queue();
    move_point({-48, -40}, 1500, {.drive_max_volt_pct = 40}).queue();
    turn_point({-48, -24}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({-48, -24}, 1200, {.reverse = true})
        .event(within({-48, -24}, 10.0, [] { intake.score(); }))
        .queue();
    wait(1000)
        .start([]() { intake.score(); })
        .queue();
    turn_point({-47, -48}, 300).queue();
    move_point({-47, -48}, 500, {.cutoff = 3.0, .drive_max_volt_pct = 50}).queue();
    move_pose({-47, -54}, 180, 1000, {.max_vel_pct = 30})
        .event(start([]() { intake.load(); }))
        .queue();
    turn_heading(180, 300).queue();
    move_time(4000, 4000, 500).queue();
    shimmy();
    move_time(-4000, -4000, 300).queue();
    turn_point({-24, -24}, 500, {.reverse = true, .cutoff = 5.0}).queue();
    move_point({-24, -24}, 1500, {.reverse = true, .cutoff = 6.0, .drive_max_volt_pct = 50}).queue();
    move_pose({-8, -8}, -135, 2000, {.reverse = true, .max_vel_pct = 30})
        .event(start([]() { intake.set(-12000, -6000); }))
        .event(elapsed(200, []() { intake.stop(); middle_piston.set_value(true); }))
        .event(elapsed(800, []() { intake.set_top_velocity(200); intake.set_bottom(12000); }))
        .queue();
    turn_point({0, 0}, 300, {.reverse = true}).queue();
    wait(1000)
        .event(start([]() { loader_piston.set_value(false); }))
        .queue();
    move_point({-36, -60}, 1500, {.cutoff = 5.0}).queue();
    turn_point({-20, -63}, 500, {.reverse = true}).queue();
    move_pose({-18, -63}, -90, 2000, {.reverse = true, .max_vel_pct = 35}).queue();
    turn_point({18, -65}, 500, {.reverse = true}).queue();
    move_time(-7000, -8000, 2000)
        .seq({
            ConditionalEvent(
                []() { return floor_optical.get_color(RED); },
                []() { loader_piston.set_value(false); }),
            ConditionalEvent(
                []() { return floor_optical.get_color(TILE); },
                []() { Miku.move_voltage(-4000, -4000); }),
        })
        .end_seq([]() { return floor_optical.get_color(RED); })
        .queue();

        // */

}

