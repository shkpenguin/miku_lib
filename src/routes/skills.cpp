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
    {18, -20, 0},
    {18, -20, 60},
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
    move_point({18, -18}, 800, {.async = true});
    wait_until_within({18, -18}, 1.0);
    // set_intake_velocity(-300);
    intake.move_voltage(-10000);
    wait_until_done();
    turn_point({0, 0}, 300);
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
    move_point({48, 32}, 2000, {.async = true, .angular_speed = 90});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    wait_until_within({48, 32}, 4.0);
    set_lock(true);
    intake.move_voltage(5000);
    wait_until_done();
    pros::delay(1500);
    intake.move_voltage(12000);
    pros::delay(200);
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
    while(front.distance_sensor.get_distance() > 1100) pros::delay(20);
    pros::delay(300);
    stop_motors();
    start_odom({-24, 66, get_pose().theta});
    initialize_particles_uniform({-32, 60}, 16.0);
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
    /*
    move_time(-12000, 200);
    move_point({-48, 32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    set_lock(true);
    wait_until_within({-48, 32}, 3.0);
    intake.move_voltage(12000);
    wait_until_done();
    pros::delay(1500);
    move_point({-48, 52}, 1200, {.reverse = true, .cutoff = 3.0});
    set_lock(false);
    turn_heading(135, 600, {.cutoff = 5.0});
    ramsete(skills_1_reverse.get_waypoints(), 3000, {.end_cutoff = 2.0});
    set_intake_velocity(-400);
    pros::delay(1800);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    move_point({-27, 27}, 1200, {.reverse = true});
    turn_heading(180, 1000, {.cutoff = 3.0});
    ramsete(skills_2_reverse.get_waypoints(), 3000, {.end_cutoff = 3.0});
    turn_point({-15, -15}, 1000);
    set_hood(false);
    ramsete(skills_2_1_reverse.get_waypoints(), 2000, {.async = true, .end_cutoff = 2.0});
    pros::delay(200);
    set_intake_velocity(400);
    wait_until_done();
    set_lock(true);
    pros::delay(1000);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    set_lock(false);
    set_hood(true);
    ramsete(skills_3_reverse.get_waypoints(), 3000, {.reverse = true});
    turn_heading(180, 1000, {.async = true});
    wait_until_within(180, 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({-48, -59}, 1000);
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1200);
    // set_wheel_tracking(true);
    move_time(-12000, 200);
    move_point({-48, -32}, 2000, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    set_lock(true);
    wait_until_within({-48, -32}, 3.0);
    set_intake_velocity(400);
    wait_until_done();
    pros::delay(1500);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    move_point({-48, -60}, 1200, {.reverse = true, .cutoff = 3.0});
    turn_point({-36, -64}, 1000);
    ramsete(skills_4_reverse.get_waypoints(), 3000, {.end_cutoff = 2.0});
    set_wheel_tracking(false);
    move_motors(6000, 6000);
    Timer park_timer(1000);
    while(front.distance_sensor.get_distance() > 1500 && !park_timer.isDone()) pros::delay(10);
    stop_motors();

    // */
}