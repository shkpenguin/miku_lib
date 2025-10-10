#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath right_9ball_1 = {
    {8, -48, 0},
    {8, -48, 70},
    {23, -24, 40},
    {38, -14, 40},
    {48, -7, 20},
    {48, -7, 0}
};

BezierPath right_9ball_2 = {
    {48, -7, 0},
    {48, -7, 30},
    {40, -36, 40},
    {48, -48, 30},
    {48, -48, 0}
};

std::vector<std::reference_wrapper<BezierPath>> right_9ball_paths = {
    std::ref(right_9ball_1),
    std::ref(right_9ball_2)
};

void pre_right_9ball() {

    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void right_9ball() {

    right_9ball_1.calculate_waypoints();
    right_9ball_2.calculate_waypoints();

    intake.move_voltage(12000);
    ramsete(right_9ball_1.get_waypoints(), 3000, {.cutoff = 3.0, .end_cutoff = 3.0});
    ramsete(right_9ball_2.get_waypoints(), 1800, {.reverse = true, .angular_weight = 0.01, .end_cutoff = 2.0});
    set_hood(true);
    intake.move_voltage(0);
    set_lock(true);
    // turn_heading(0, 500, {.cutoff = 10.0});
    // move_pose({49, -24, 0}, 1000);
    move_point({48, -26}, 1000);
    intake.move_voltage(12000);
    pros::delay(1200);
    move_pose({48, -60, 180}, 1200, {.async = true, .distance_weight = 2.5, .angular_weight = 2.5});
    pros::delay(200);
    set_lock(false);
    set_loading(true);
    wait_until_done();
    move_point({48, -65}, 1000);
    move_time(5000, 1000);
    intake.move_voltage(0);
    set_lock(true);
    move_pose({48, -28, 0}, 1800, {.async = true, .distance_weight = 2.5, .angular_weight = 2.0});
    pros::delay(200);
    set_loading(false);
    wait_until_done();
    intake.move_voltage(12000);
    pros::delay(700);
    intake.move_voltage(0);
    move_time(3000, 3000);

}