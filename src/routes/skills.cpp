#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath skills_1 = {
    {48, -54, 0},
    {48, -54, 50},
    {32, -30, 30},
    {24, -24, 20},
    {16, -16, 0},
    {16, -16, 0}
};

BezierPath skills_2 = {
    {28, -25, 0},
    {28, -25, 75},
    {26, 0, 40},
    {25, 12, 20},
    {24, 24, 0},
    {24, 24, 0}
};

BezierPath skills_2_1 = {
    {26, 23, 0},
    {26, 23, 50},
    {18, 18, 20},
    {15, 15, 0},
    {15, 15, 0}
};

BezierPath skills_3 = {
    {15, 15, 0},
    {15, 15, 100},
    {25, 25, 70},
    {36, 36, 50},
    {50, 48, 20},
    {50, 48, 0}
};

BezierPath skills_4 = {
    {48, 60, 0},
    {48, 60, 70},
    {36, 64, 30},
    {24, 66, 20},
    {18, 66, 0},
    {18, 66, 0}
};

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {
    std::ref(skills_1),
    std::ref(skills_2),
    std::ref(skills_2_1),
    std::ref(skills_3),
    std::ref(skills_4)
};

void pre_skills() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    set_hood(true);
}

void skills() {
    intake.move_voltage(12000);
    move_point({45, -48}, 1200, {.cutoff = 2.0});
    turn_heading(180, 800, {.async = true, .cutoff = 10.0});
    wait_until_within(180, 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, -59}, 1000);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1200);
    // set_wheel_tracking(true);
    move_time(-12000, 200);
    move_point({48, -32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    set_lock(true);
    wait_until_within({48, -32}, 3.0);
    intake.move_voltage(12000);
    wait_until_done();
    pros::delay(1500);
    move_point({48, -52}, 1200, {.reverse = true, .cutoff = 3.0});
    set_lock(false);
    turn_heading(-45, 600, {.cutoff = 5.0});
    ramsete(skills_1.get_waypoints(), 3000, {.end_cutoff = 2.0});
    set_intake_velocity(-400);
    pros::delay(1800);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    move_point({27, -27}, 1200, {.reverse = true});
    turn_heading(0, 1000, {.cutoff = 3.0});
    ramsete(skills_2.get_waypoints(), 3000, {.end_cutoff = 3.0});
    turn_point({15, 15}, 1000);
    set_hood(false);
    ramsete(skills_2_1.get_waypoints(), 2000, {.async = true, .end_cutoff = 2.0});
    pros::delay(200);
    set_intake_velocity(400);
    wait_until_done();
    set_lock(true);
    pros::delay(1000);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    set_lock(false);
    set_hood(true);
    ramsete(skills_3.get_waypoints(), 3000, {.reverse = true});
    turn_heading(0, 1000, {.async = true});
    wait_until_within(0, 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, 59}, 1000);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1200);
    // set_wheel_tracking(true);
    move_time(-12000, 200);
    move_point({48, 32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    set_lock(true);
    wait_until_within({48, 32}, 3.0);
    intake.move_voltage(12000);
    wait_until_done();
    pros::delay(1500);
    move_point({48, 60}, 1200, {.reverse = true, .cutoff = 3.0});
    turn_point({36, 64}, 1000);
    ramsete(skills_4.get_waypoints(), 3000, {.end_cutoff = 2.0});
    set_wheel_tracking(false);
    move_motors(6000, 6000);
    Timer barrier_timer(1800);
    while(front.distance_sensor.get_distance() > 1100 && !barrier_timer.isDone()) pros::delay(10);
    stop_motors();
    set_wheel_tracking(true);
    start_odom({-18, 66, get_pose().theta});
    initialize_particles_uniform({-18, 64}, 6.0);
    set_wheel_tracking(true);
    move_point({-48, 48}, 3000);

}