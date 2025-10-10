#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath right_sawp_1 = {
    {6.5, -48, 0},
    {6.5, -48, 100},
    {24, -48, 80},
    {42, -48, 15},
    {48, -48, 0},
    {48, -48, 0}
};

BezierPath right_sawp_2 = {
    {48, -48, 40},
    {48, -48, 40},
    {48, -64, 30},
    {48, -64, 0}
};

BezierPath right_sawp_3 = {
    {48, -60, 0},
    {48, -60, 90},
    {48, -34, 0},
    {48, -34, 0}
};

BezierPath right_sawp_4 = {
    {48, -36, 0},
    {48, -36, 40},
    {24, -24, 20},
    {18, -18, 20},
    {15, -15, 0},
    {15, -15, 0}
};

BezierPath right_sawp_5 = {
    {20, -24, 0},
    {20, -24, 100},
    {0, -21, 50},
    {-12, -21, 15},
    {-24, -21, 0},
    {-24, -21, 0}
};

BezierPath right_sawp_6 = {
    {-24, -21, 0},
    {-24, -21, 50},
    {-18, -18, 20},
    {-14, -14, 0},
    {-14, -14, 0}
};

std::vector<std::reference_wrapper<BezierPath>> right_sawp_paths = {
    std::ref(right_sawp_1),
    std::ref(right_sawp_2),
    std::ref(right_sawp_3),
    std::ref(right_sawp_4),
    std::ref(right_sawp_5),
    std::ref(right_sawp_6)
};

void pre_right_sawp() {

    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);
    set_hood(true);

}

void right_sawp() {

    intake.move_voltage(12000);

    ramsete(right_sawp_1.get_waypoints(), 3000);
    wait_until_done();
    pros::delay(300);
    set_loading(true);
    turn_heading(180, 1200);
    ramsete(right_sawp_2.get_waypoints(), 600);
    pros::delay(400);
    move_time(-4000, 400);
    move_time(0, 200);
    turn_heading(0, 1000);
    set_loading(false);

    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    ramsete(right_sawp_3.get_waypoints(), 600);
    wait_until_within({48, -34}, 6.0);
    set_lock(true);
    wait_until_done();
    pros::delay(1200);
    set_lock(false);
    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    move_time(-6000, 300);
    turn_heading(-40, 700);
    ramsete(right_sawp_4.get_waypoints(), 2200);
    intake.move_voltage(-8000);
    pros::delay(1500);
    intake.move_voltage(12000);
    move_time(-6000, 200);

    turn_heading(-90, 700);
    ramsete(right_sawp_5.get_waypoints(), 2000);
    turn_heading(50, 700);
    intake.move_voltage(4000);
    ramsete(right_sawp_6.get_waypoints(), 2000);
    move_time(2000, 300);
    set_hood(false);
    set_lock(true);
    move_time(2000, 800);

}