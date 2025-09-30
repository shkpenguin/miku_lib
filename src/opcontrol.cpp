#include "config.h"
#include "main.h"
#include "timer.h"
#include "mcl.h"
#include "misc.h"
#include "notif.h"
#include "motions.h"
#include "autons.h"
#include "util.h"

enum class DriveMode {
    TANK = 0,
    ARCADE = 1
};

int curve(int pos) {
    if(fabs(pos) <= 5) return 0;
    return (pos * pos * pos * 0.00337827447) + (pos * 40);
}

void tank(int left, int right) {
    left_motors.move_voltage(curve(left));
    right_motors.move_voltage(curve(right));
}

void arcade(int throttle, int turn) {
    int left = throttle + turn;
    int right = throttle - turn;

    left_motors.move_voltage(curve(left));
    right_motors.move_voltage(curve(right));
}

DriveMode driveMode = DriveMode::TANK;

bool hood_up = false;
bool lock = false;
bool loading = false;
bool descore = false;

void set_lock(bool state) {
    lock = state;
    lock_piston.set_value(lock);
}

void set_loading(bool state) {
    loading = state;
    loader_piston.set_value(loading);
}

void set_hood(bool state) {
    hood_up = state;
    hood_piston.set_value(hood_up);
}

void set_descore(bool state) {
    descore = state;
    descore_piston.set_value(descore);
}

bool get_hood() {
    return hood_up;
}

bool get_loading() {
    return loading;
}

bool get_lock() {
    return lock;
}

bool get_descore() {
    return descore;
}

void opcontrol() {

    rumble_timer.resume();
    // autonomous_task.remove();
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

    int count = 0;

    while (true) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            driveMode = (driveMode == DriveMode::TANK) ? DriveMode::ARCADE : DriveMode::TANK;
            master.rumble("."); // Short vibration to indicate mode change
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            controller_display->suspend();
            tune_lut();
            break;
        }

        if(driveMode == DriveMode::TANK) {
            int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            tank(left, right);
        } else if(driveMode == DriveMode::ARCADE) {
            int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            arcade(throttle, turn);
        }

        //r1 toggle hood
        //r2 intake
        //shift r2 intake

        bool shift1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool shift2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool intaking = false;
        bool loader_changed = false;

        if(!shift1 && !shift2) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move_voltage(12000);
                intaking = true;
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                lock = !lock;
                lock_piston.set_value(lock);
            }
        }

        else if(shift1) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move_voltage(-12000);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                descore = !descore;
                descore_piston.set_value(descore);
            }
        }

        if(shift2 && hood_up && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            loading = !loading;
            loader_piston.set_value(loading);
            loader_changed = true;
        }

        bool hood_toggle_pressed =
        (shift2 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) ||
        (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

        if((!loading || !hood_up) && 
           (hood_toggle_pressed)) {
            hood_up = !hood_up;
            hood_piston.set_value(hood_up);
        }

        if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(0);
        }

        pros::delay(10);
    }
}