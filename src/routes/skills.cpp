#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath skills_1 = {
    {48, -40, 0},
    {48, -40, 60},
    {36, -31, 15},
    {24, -24, 10},
    {24, -24, 0}
};

BezierPath skills_2 = {
    {18, -18, 0},
    {18, -18, 70},
    {21, 0, 40},
    {23, 12, 20},
    {24, 20, 10},
    {25, 27, 0},
    {25, 27, 0}
};

BezierPath skills_3 = {
    {15, 15, 0},
    {15, 15, 100},
    {25, 25, 70},
    {36, 36, 50},
    {51, 46, 0},
    {51, 46, 0}
};

BezierPath skills_4 = {
    {48, 58, 0},
    {48, 58, 80},
    {36, 61, 40},
    {24, 64, 20},
    {18, 64, 0},
    {18, 64, 0}
};

BezierPath skills_1_reversed = {
    {-48, 40, 0},
    {-48, 40, 60},
    {-36, 31, 15},
    {-24, 24, 10},
    {-24, 24, 0}
};

BezierPath skills_2_reversed = {
    {-18, 18, 0},
    {-18, 18, 70},
    {-21, 0, 40},
    {-23, -12, 20},
    {-24, -20, 10},
    {-25, -27, 0},
    {-25, -27, 0}
};

BezierPath skills_3_reversed = {
    {-15, -15, 0},
    {-15, -15, 100},
    {-25, -25, 70},
    {-36, -36, 50},
    {-51, -46, 0},
    {-51, -46, 0}
};

BezierPath skills_4_reversed = {
    {-48, -58, 0},
    {-48, -58, 60},
    {-36, -63, 40},
    {-24, -64, 20},
    {-18, -64, 0},
    {-18, -64, 0}
};

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {
    std::ref(skills_1),
    std::ref(skills_2),
    std::ref(skills_3),
    std::ref(skills_4),
    std::ref(skills_1_reversed),
    std::ref(skills_2_reversed),
    std::ref(skills_3_reversed),
    std::ref(skills_4_reversed)
};

void pre_skills() {
    set_hood(true);
    set_lock(true);
}

void skills() {
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
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1200);
    // set_wheel_tracking(true);
    move_time(-12000, 100);
    set_loading(false);
    turn_point({48, -31}, 700);
    intake.move_voltage(0);
    move_point({48, -31}, 1300, {.async = true});
    wait_until_within({48, -31}, 6.0);
    set_lock(false);
    intake.move_voltage(12000);
    wait_until_done();
    swing_point({48, -20}, 500);
    pros::delay(500);
    move_point({48, -40}, 500, {.reverse = true, .cutoff = 1.0});
    set_lock(true);
    turn_point({24, -24}, 500);
    ramsete(skills_1.get_waypoints(), 1500, {.async = true, .end_cutoff = 3.0});
    pros::delay(300);

    move_point({16, -16}, 800);
    // set_intake_velocity(-300);
    wait_until_done();
    intake.move_voltage(-9000);
    pros::delay(100);
    turn_heading(-45, 500);
    pros::delay(1400);
    move_time(-8000, 150);
    // set_intake_tbh(false);
    intake.move_voltage(12000);
    set_max_distance_error(4.0);
    turn_point({24, 24}, 600);
    ramsete(skills_2.get_waypoints(), 1800, {.end_cutoff = 3.0});
    intake.move_voltage(0);
    turn_point({14, 14}, 700);
    set_hood(false);
    set_lock(false);
    set_loading(true);
    move_point({14, 14}, 700, {.async = true});
    wait_until_within({14, 14}, 1.0);
    intake.move_voltage(7000);
    wait_until_done();
    pros::delay(100);
    turn_point({0, 0}, 500);
    pros::delay(500);
    intake.move_voltage(12000);
    pros::delay(300);
    set_hood(true);
    set_max_distance_error(8.0);
    set_loading(false);
    ramsete(skills_3.get_waypoints(), 1200, {.reverse = true, .async = true});
    pros::delay(300);
    set_lock(true);
    wait_until_done();
    turn_point({47, 59}, 1000, {.async = true});
    wait_until_within(get_pose().angle_to({47, 59}), 40.0);
    set_loading(true);
    wait_until_done();
    move_point({47, 59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 210) break;
        pros::delay(10);
    }
    wait_until_done();
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1400);
    // set_wheel_tracking(true);
    move_time(-12000, 100);
    set_loading(false);
    turn_point({48, 31}, 700);
    intake.move_voltage(0);
    move_point({48, 31}, 1300, {.async = true});
    pros::delay(200);
    set_loading(false);
    pros::delay(200);
    intake.move_voltage(0);
    wait_until_within({48, 31}, 6.0);
    set_lock(false);
    intake.move_voltage(12000);
    wait_until_done();
    swing_point({48, 20}, 500);
    pros::delay(400);
    intake.move_voltage(12000);
    pros::delay(400);
    intake.move_voltage(0);
    move_point({48, 54}, 800, {.reverse = true, .cutoff = 2.0});
    turn_point({36, 64}, 600);
    ramsete(skills_4.get_waypoints(), 900);
    set_hood(false);
    set_loading(true);
    set_wheel_tracking(false);
    intake.move_voltage(12000);
    move_motors(6000, 6000);
    while(!optical.get_color(BLUE)) pros::delay(10);
    while(!optical.get_color(TILE)) pros::delay(10);
    move_motors(4500, 4500);
    set_loading(false);
    while(!optical.get_color(BLUE)) pros::delay(10);
    while(!optical.get_color(TILE)) pros::delay(10);
    pros::delay(100);
    stop_motors();
    start_odom({-24, 66, get_pose().theta});
    initialize_particles_uniform({-24, 60}, 24.0);
    set_min_odom_noise(0.5);
    set_wheel_tracking(true);
    set_hood(true);
    set_max_sensor_stdev(0.5);
    pros::delay(300);
    move_point({-47, 44}, 2500, {.async = true});
    pros::delay(700);
    set_lock(true);
    wait_until_done();
    set_min_odom_noise(0.05);
    set_max_sensor_stdev(100.0);
    turn_point({-47, 59}, 800, {.async = true});
    wait_until_within(get_pose().angle_to({-47, 59}), 40.0);
    set_loading(true);
    wait_until_done();
    pros::delay(100);
    move_point({-47, 59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 210) break;
        pros::delay(10);
    }
    wait_until_done();
    set_min_odom_noise(0.05);
    pros::delay(1200);
    move_time(-12000, 100);
    set_loading(false);
    turn_point({-48, 31}, 700);
    intake.move_voltage(0);
    move_point({-48, 31}, 1300, {.async = true});
    wait_until_within({-48, 31}, 6.0);
    set_lock(false);
    intake.move_voltage(12000);
    wait_until_done();
    swing_point({-48, 20}, 500);
    pros::delay(500);

    // mirrored starts here
    set_max_distance_error(3.0);
    move_point({-48, 40}, 500, {.reverse = true, .cutoff = 1.0});
    set_lock(true);
    turn_point({-24, 24}, 500);
    ramsete(skills_1_reversed.get_waypoints(), 1500, {.async = true, .end_cutoff = 3.0});
    pros::delay(300);
    move_point({-16, 16}, 800);
    // set_intake_velocity(-300);
    wait_until_done();
    intake.move_voltage(-8000);
    pros::delay(100);
    turn_heading(135, 500);
    pros::delay(1400);
    move_time(-8000, 150);
    // set_intake_tbh(false);
    intake.move_voltage(12000);
    turn_point({-24, -24}, 600);
    ramsete(skills_2_reversed.get_waypoints(), 1800, {.end_cutoff = 3.0});
    intake.move_voltage(0);
    turn_point({-14, -14}, 700);
    set_hood(false);
    set_lock(false);
    set_loading(true);
    move_point({-14, -14}, 700, {.async = true});
    wait_until_within({-14, -14}, 1.0);
    intake.move_voltage(9000);
    wait_until_done();
    pros::delay(100);
    turn_point({0, 0}, 500);
    pros::delay(700);
    intake.move_voltage(12000);
    set_max_distance_error(8.0);
    set_loading(false);
    ramsete(skills_3_reversed.get_waypoints(), 1200, {.reverse = true});
    set_hood(true);
    set_lock(true);
    turn_point({-47, -59}, 1000, {.async = true});
    wait_until_within(get_pose().angle_to({-47, -59}), 40.0);
    set_loading(true);
    wait_until_done();
    pros::delay(100);
    move_point({-47, -59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 210) break;
        pros::delay(10);
    }
    wait_until_done();
    // set_wheel_tracking(false);
    // move_time(6000, 1500);
    pros::delay(1500);
    // set_wheel_tracking(true);
    move_time(-12000, 100);
    set_loading(false);
    turn_point({-48, -31}, 700);
    intake.move_voltage(0);
    move_point({-48, -31}, 1300, {.async = true});
    wait_until_within({-48, -31}, 6.0);
    set_lock(false);
    intake.move_voltage(12000);
    wait_until_done();
    swing_point({-48, -20}, 500);
    pros::delay(400);
    intake.move_voltage(12000);
    pros::delay(400);
    intake.move_voltage(0);
    move_point({-48, -54}, 800, {.reverse = true, .cutoff = 2.0});
    turn_point({-36, -63}, 700);
    ramsete(skills_4_reversed.get_waypoints(), 800);
    set_hood(false);
    set_wheel_tracking(false);
    intake.move_voltage(12000);
    set_loading(true);
    move_motors(7500, 7500);
    while(!optical.get_color(RED)) pros::delay(10);
    set_loading(false);
    while(!optical.get_color(TILE)) pros::delay(10);
    pros::delay(50);
    stop_motors();

    // */
}