#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath skills_1 = {
    {48, -54, 0},
    {48, -54, 50},
    {32, -30, 30},
    {24, -24, 25},
    {13, -13, 0},
    {13, -13, 0}
};

BezierPath skills_2 = {
    {24, -24, 0},
    {24, -24, 100},
    {24, 0, 70},
    {24, 12, 20},
    {24, 26, 0},
    {24, 26, 0}
};

BezierPath skills_3 = {
    {15, 15, 0},
    {15, 15, 100},
    {24, 24, 60},
    {36, 32, 30},
    {48, 36, 0}
};

BezierPath skills_4 = {
    {48, 20, 0},
    {48, 31, 70},
    {36, 60, 30},
    {24, 65, 20},
    {18, 65, 0},
    {18, 65, 0}
};

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {
    std::ref(skills_1),
    std::ref(skills_2),
    std::ref(skills_3),
    std::ref(skills_4)
};

void pre_skills() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    set_hood(true);
}

void skills() {
    intake.move_voltage(12000);
    move_pose({48, -48, 90}, 1200, {.cutoff = 2.0, .angular_weight = 0});

    /*
    turn_heading(180, 800, {.async = true, .cutoff = 10.0});
    wait_until_within(180, 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, -61}, 1000);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1000);
    // set_wheel_tracking(true);
    move_pose({48, -31, 0}, 2000, {.async = true, .angular_weight = 1.0});
    pros::delay(200);
    set_loading(false);
    wait_until_done();
    set_lock(true);
    pros::delay(1500);
    move_point({48, -54}, 1200, {.reverse = true, .cutoff = 3.0});
    set_lock(false);
    turn_heading(-45, 600, {.cutoff = 5.0});
    ramsete(skills_1.get_waypoints(), 3000, {.end_cutoff = 3.0});
    set_intake_velocity(-400);
    pros::delay(1800);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    move_point({24, -24}, 1200, {.reverse = true, .cutoff = 3.0});
    turn_heading(0, 800, {.cutoff = 5.0});
    ramsete(skills_2.get_waypoints(), 3000, {.end_cutoff = 3.0});
    turn_point({15, 15}, 1000);
    set_hood(false);
    move_point({15, 15}, 800);
    set_lock(true);
    pros::delay(1000);
    set_lock(false);
    set_hood(true);
    ramsete(skills_3.get_waypoints(), 3000, {.reverse = true});
    turn_point({48, 61}, 1000, {.async = true});
    wait_until_within(0, 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, 61}, 1000);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1000);
    // set_wheel_tracking(true);
    move_pose({48, 31, 0}, 2000, {.reverse = true, .async = true, .angular_weight = 1.0});
    pros::delay(200);
    set_loading(false);
    wait_until_done();
    set_lock(true);
    pros::delay(1500);
    ramsete(skills_4.get_waypoints(), 3000, {.reverse = true, .end_cutoff = 3.0});
    */

}