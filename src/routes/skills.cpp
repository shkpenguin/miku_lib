#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath skills_1 = {
    {48, -54, 0},
    {48, -54, 60},
    {32, -32, 30},
    {24, -24, 20},
    {18, -18, 0},
    {0, 0, 0}
};

BezierPath skills_2 = {
    {28, -25, 0},
    {28, -25, 60},
    {26, 0, 40},
    {25, 12, 30},
    {24, 27, 0},
    {24, 27, 0}
};

BezierPath skills_2_1 = {
    {25, 25, 0},
    {25, 25, 50},
    {18, 18, 20},
    {15, 15, 0},
    {15, 15, 0}
};

BezierPath skills_3 = {
    {15, 15, 0},
    {15, 15, 100},
    {25, 25, 70},
    {36, 36, 50},
    {50, 48, 0},
    {50, 48, 0}
};

BezierPath skills_4 = {
    {48, 58, 0},
    {48, 58, 60},
    {36, 62, 40},
    {24, 64, 20},
    {21, 64, 0},
    {21, 64, 0}
};

BezierPath skills_1_reverse = {
    {-48, 54, 0},
    {-48, 54, 50},
    {-32, 30, 30},
    {-24, 24, 20},
    {-16, 16, 0},
    {-16, 16, 0}
};

BezierPath skills_2_reverse = {
    {-28, 25, 0},
    {-28, 25, 75},
    {-26, 0, 40},
    {-25, -12, 20},
    {-24, -24, 0},
    {-24, -24, 0}
};

BezierPath skills_2_1_reverse = {
    {-26, -23, 0},
    {-26, -23, 50},
    {-18, -18, 20},
    {-15, -15, 0},
    {-15, -15, 0}
};

BezierPath skills_3_reverse = {
    {-15, -15, 0},
    {-15, -15, 100},
    {-25, -25, 70},
    {-36, -36, 50},
    {-50, -48, 20},
    {-50, -48, 0}
};

BezierPath skills_4_reverse = {
    {-48, -60, 0},
    {-48, -60, 70},
    {-36, -64, 30},
    {-24, -66, 20},
    {-18, -66, 0},
    {-18, -66, 0}
};

std::vector<std::reference_wrapper<BezierPath>> skills_paths = {
    std::ref(skills_1),
    std::ref(skills_2),
    std::ref(skills_2_1),
    std::ref(skills_3),
    std::ref(skills_4),
    std::ref(skills_1_reverse),
    std::ref(skills_2_reverse),
    std::ref(skills_2_1_reverse),
    std::ref(skills_3_reverse),
    std::ref(skills_4_reverse)
};

void pre_skills() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    set_hood(true);
}

void skills() {
    intake.move_voltage(12000);
    move_point({44, -48}, 1200, {.cutoff = 2.0});
    turn_point({48, -59}, 800, {.async = true, .cutoff = 15.0});
    wait_until_within(get_pose().angle_to({48, -59}), 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, -59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 200) break;
        pros::delay(10);
    }
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
    wait_until_within({48, -32}, 2.0);
    intake.move_voltage(12000);
    wait_until_done();
    pros::delay(1000);
    move_point({48, -50}, 1200, {.reverse = true, .cutoff = 3.0});
    set_lock(false);
    turn_heading(-45, 600, {.cutoff = 5.0});
    ramsete(skills_1.get_waypoints(), 3000, {.end_cutoff = 2.0});
    intake.move_voltage(-12000);
    turn_point({0, 0}, 300);
    pros::delay(1800);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    move_point({27, -27}, 1200, {.reverse = true, .cutoff = 2.0});
    turn_point({24, 24}, 1000, {.cutoff = 5.0});
    ramsete(skills_2.get_waypoints(), 3000, {.cutoff = 2.0});
    turn_point({0, 0}, 1000);
    set_hood(false);
    ramsete(skills_2_1.get_waypoints(), 2000, {.async = true, .end_cutoff = 3.0});
    pros::delay(200);
    set_intake_velocity(400);
    wait_until_done();
    turn_point({0, 0}, 1000);
    set_lock(true);
    pros::delay(1000);
    set_intake_tbh(false);
    intake.move_voltage(12000);
    set_hood(true);
    ramsete(skills_3.get_waypoints(), 2000, {.reverse = true, .async = true});
    pros::delay(300);
    set_lock(false);
    wait_until_done();
    turn_point({48, 59}, 1000, {.async = true, .cutoff = 15.0});
    wait_until_within(get_pose().angle_to({48, 59}), 20.0);
    set_loading(true);
    wait_until_done();
    pros::delay(300);
    move_point({48, 59}, 1000, {.async = true});
    while(get_motion_running()) {
        if(front.distance_sensor.get_distance() < 200) break;
        pros::delay(10);
    }
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
    wait_until_within({48, 32}, 2.0);
    set_intake_velocity(400);
    wait_until_done();
    pros::delay(1500);
    set_intake_tbh(false);
    intake.move_voltage(0);
    move_point({48, 54}, 1200, {.reverse = true, .cutoff = 2.0});
    turn_point({36, 60}, 1000);
    set_lock(true);
    ramsete(skills_4.get_waypoints(), 2000);
    set_hood(false);
    set_wheel_tracking(false);
    intake.move_voltage(12000);
    move_motors(6000, 6000);
    pros::delay(750);
    intake.move_voltage(0);
    Timer barrier_timer(1500);
    int dist_exit_timer = 0;
    while(!barrier_timer.isDone()) {
        if(dist_exit_timer++ > 50) break;
        if(front.distance_sensor.get_distance() < 1100) dist_exit_timer += 10;
        pros::delay(10);
    }
    stop_motors();
    set_wheel_tracking(true);
    start_odom({-18, 66, get_pose().theta});
    initialize_particles_uniform({-18, 60}, 8.0);
    set_wheel_tracking(true);
    set_hood(true);
    // move_point({-48, 48}, 3000, {.max_speed = 6000});
    /*
    turn_heading(0, 1000, {.async = true});
    wait_until_within(0, 20.0);
    set_loading(true);
    wait_until_done();
    move_point({-48, 59}, 1000);
    pros::delay(1200);
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