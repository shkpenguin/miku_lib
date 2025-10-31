#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath right_sawp_1 = {
    {48, -40, 0},
    {48, -40, 60},
    {36, -31, 15},
    {24, -24, 10},
    {24, -24, 0}
};

BezierPath right_sawp_2 = {
    {18, -18, 0},
    {18, -18, 75},
    {0, -24, 50},
    {-18, -24, 20},
    {-24, -24, 10},
    {-24, -24, 0}
};

std::vector<std::reference_wrapper<BezierPath>> right_sawp_paths = {
    std::ref(right_sawp_1),
    std::ref(right_sawp_2)
};

void pre_right_sawp() {

    set_hood(true);
    set_lock(true);

}

void right_sawp() {

    intake.move_voltage(12000);
    move_point({48, -48}, 800);
    pros::delay(200);
    turn_point({47, -59}, 700, {.async = true});
    wait_until_within(180, 40.0);
    set_loading(true);
    wait_until_done();
    move_point({47, -59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 210) break;
        pros::delay(10);
    }
    wait_until_done();
    pros::delay(300);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    // set_wheel_tracking(true);
    move_time(-12000, 100);
    set_loading(false);
    turn_point({48, -31}, 700);
    intake.move_voltage(0);
    move_point({48, -31}, 1300, {.async = true});
    wait_until_within({48, -31}, 6.0);
    lock_piston.set_value(false);
    intake.move_voltage(12000);
    wait_until_done();
    swing_point({48, -20}, 500);
    move_point({48, -40}, 500, {.reverse = true, .cutoff = 1.0});
    lock_piston.set_value(true);
    turn_point({24, -24}, 500);
    ramsete(right_sawp_1.get_waypoints(), 1500, {.async = true, .end_cutoff = 3.0});
    pros::delay(300);

    move_point({16, -16}, 800);
    // set_intake_velocity(-300);
    wait_until_done();
    intake.move_voltage(-9000);
    turn_heading(315, 500);
    pros::delay(1000);
    
    /*
    move_time(-8000, 200);

    intake.move_voltage(12000);

    turn_point({0, -24}, 700);
    ramsete(right_sawp_2.get_waypoints(), 1500, {.end_cutoff = 3.0});

    turn_point({-14, -14}, 700);
    intake.move_voltage(0);
    set_hood(false);
    set_lock(false);
    set_loading(true);
    move_point({-14, -14}, 700, {.async = true});
    wait_until_within({-14, -14}, 1.0);
    intake.move_voltage(9000);
    wait_until_done();
    turn_point({0, 0}, 500);
    pros::delay(800);

    /*
    turn_point({0, -24}, 700);
    ramsete(right_sawp_2.get_waypoints(), 1800, {.end_cutoff = 3.0});
    set_hood(false);
    turn_point({-14, -14}, 700);
    intake.move_voltage(6000);
    move_point({-14, -14}, 700, {.async = true});
    wait_until_within({-14, -14}, 1.0);
    set_lock(true);
    wait_until_done();
    turn_point({0, 0}, 500);
    // */

}