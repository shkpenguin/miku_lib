#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath skills_1 = {
    {48, -38, 0},
    {48, -38, 60},
    {36, -31, 15},
    {24, -24, 10},
    {24, -24, 0}
};

BezierPath skills_2 = {
    {17, -17, 0},
    {17, -17, 60},
    {21, 0, 35},
    {23, 12, 20},
    {24, 20, 15},
    {25, 27, 10},
    {25, 27, 0}
};

BezierPath skills_3 = {
    {15, 15, 0},
    {15, 15, 100},
    {25, 25, 70},
    {36, 36, 50},
    {54, 42, 0},
    {54, 42, 0}
};

BezierPath skills_4 = {
    {48, 58, 0},
    {48, 58, 60},
    {36, 63, 40},
    {24, 64, 20},
    {18, 64, 0},
    {18, 64, 0}
};

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {
    std::ref(skills_1),
    std::ref(skills_2),
    std::ref(skills_3),
    std::ref(skills_4),
};

void pre_skills() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    set_hood(true);
}

void skills() {
    intake.move_voltage(12000);
    move_point({47, -48}, 1200, {.cutoff = 1.5});
    turn_point({48, -59}, 800, {.async = true, .cutoff = 5.0});
    wait_until_within(180, 40.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, -59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 200) break;
        pros::delay(10);
    }
    wait_until_done();
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
    wait_until_within({48, -32}, 4.0);
    set_lock(true);
    intake.move_voltage(12000);
    wait_until_done();
    pros::delay(1000);
    move_point({48, -38}, 500, {.reverse = true, .cutoff = 1.0});
    set_lock(false);
    turn_point({24, -24}, 700);
    ramsete(skills_1.get_waypoints(), 1500, {.async = true, .end_cutoff = 3.0});
    pros::delay(300);
    move_point({17, -17}, 800);
    // set_intake_velocity(-300);
    wait_until_done();
    intake.move_voltage(-9000);
    turn_point({0, 0}, 500);
    pros::delay(1800);
    // set_intake_tbh(false);
    intake.move_voltage(12000);
    turn_point({24, 24}, 600);
    ramsete(skills_2.get_waypoints(), 2200, {.end_cutoff = 3.0});
    intake.move_voltage(0);
    turn_point({14, 14}, 700);
    set_hood(false);
    set_lock(true);
    set_loading(true);
    move_point({14, 14}, 700, {.async = true});
    wait_until_within({14, 14}, 1.0);
    intake.move_voltage(8000);
    wait_until_done();
    turn_point({0, 0}, 500);
    pros::delay(800);
    intake.move_voltage(12000);
    pros::delay(500);
    set_hood(true);
    set_max_distance_error(8.0);
    set_loading(false);
    ramsete(skills_3.get_waypoints(), 1500, {.reverse = true, .async = true});
    pros::delay(300);
    set_lock(false);
    wait_until_done();
    turn_point({48, 72}, 1000, {.async = true, .cutoff = 5.0});
    wait_until_within(get_pose().angle_to({48, 72}), 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, 59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 200) break;
        pros::delay(10);
    }
    wait_until_done();
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1500);
    // set_wheel_tracking(true);
    move_time(-12000, 200);
    move_point({48, 32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    wait_until_within({48, 32}, 4.0);
    set_lock(true);
    intake.move_voltage(7000);
    wait_until_done();
    pros::delay(1200);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(0);
    move_point({48, 54}, 800, {.reverse = true, .cutoff = 2.0});
    turn_point({36, 60}, 700);
    set_lock(true);
    ramsete(skills_4.get_waypoints(), 1000);
    set_hood(false);
    set_wheel_tracking(false);
    intake.move_voltage(12000);
    move_time(8000, 600);
    intake.move_voltage(0);
    move_motors(4000, 4000);
    while(!optical.get_color(BLUE)) pros::delay(20);
    while(!optical.get_color(TILE)) pros::delay(20);
    pros::delay(500);
    stop_motors();
    start_odom({-24, 66, get_pose().theta});
    initialize_particles_uniform({-24, 60}, 24.0);
    set_wheel_tracking(true);
    set_hood(true);
    move_point({-48, 48}, 3000, {.max_speed = 6000});
    turn_point({-48, 59}, 800, {.async = true, .cutoff = 10.0});
    wait_until_within(get_pose().angle_to({-48, 59}), 20.0);
    intake.move_voltage(12000);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({-48, 59}, 1000);
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 200) break;
        pros::delay(10);
    }
    wait_until_done();
    pros::delay(1200);
    move_time(-12000, 200);
    move_point({-48, 32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    wait_until_within({-48, 32}, 4.0);
    set_lock(true);
    intake.move_voltage(12000);
    wait_until_done();

    // */
}