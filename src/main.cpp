#include "main.h"
#include "routes.hpp"
#include "miku/miku-api.hpp"
#include "system.hpp"
#include "fmt/core.h"
#include "macro.hpp"
#include <deque>
#include <vector>

int selected_idx = 5;
std::vector<Route> routes;

int curve(int pos) {
    if(abs(pos) <= 5) return 0;
    // return 10502.7578057 * (std::exp(0.006 * abs(pos)) - 1.0) * (pos > 0 ? 1 : -1);
    return pos * (12000 / 127);
}

void tank(int left, int right) {
    Miku.move_voltage(curve(left), curve(right));
}

void funny_tank(int left_x, int left_y, int right_x, int right_y) {
    if(abs(left_x) > 50 && abs(right_x) > 50) {
        int sign = (left_x > 0 || right_x < 0) ? 1 : -1;
        left_x = (abs(left_x) - 50) * 127 / 77;
        right_x = (abs(right_x) - 50) * 127 / 77;
        float speed = (abs(left_x) + abs(right_x)) / 2.0 * sign;
        Miku.move(speed, speed);
    } else {
        Miku.move(left_y, right_y);
    }
}

void arcade(int throttle, int turn) {
    int left = curve(throttle + turn);
    int right = curve(throttle - turn);

    Miku.move_voltage(left, right);
}

void precalculate_paths() {
    routes.push_back(Route("test route", {24, -48, -M_PI_2}, test));
    routes.push_back(Route("skills", {14, -46, M_PI_2}, skills));
    routes.push_back(Route("sawp", {0, -48, M_PI}, sawp));
    routes.push_back(Route("right rush", {14, -46, M_PI_2}, right_rush));
    routes.push_back(Route("skills mid control", {0, 0, M_PI}, skills_mid_control));
    routes.push_back(Route("left rush", {-18, -51, M_PI}, left_rush));
}

void initialize() {

    left_motors.tare_position();
    right_motors.tare_position();
    intake_top.tare_position();
    intake_bottom.tare_position();

    Miku.set_brake_mode(DEFAULT_AUTONOMOUS_BRAKE_MODE);

    intake_optical.set_led_pwm(100);
    intake_optical.set_integration_time(10);
    floor_optical.set_led_pwm(100);
    floor_optical.set_integration_time(10);

    precalculate_paths();

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

    master.clear();

    Miku.set(routes[selected_idx].start_pose);
    Miku.calibrate();
    routes[selected_idx].queue();

}

void autonomous() {

    display_pose();
    descore_piston.set_value(true);

    autonomous_system_task = new pros::Task([]() {
        pros::delay(10); // give the task time to start (idk if this does anything)
        autonomous_system_control();
    });

}

void opcontrol() {

    // /*

    motion_queue.clear();
    current_motion = nullptr;

    if(autonomous_system_task != nullptr) autonomous_system_task->remove();

    Miku.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // skills_mid_control();

    // static Gif gif("/usd/jiachenma.gif", lv_scr_act());

    pros::Task teleop_system_task([]() {
        pros::delay(10);
        teleop_system_control();
    });

}