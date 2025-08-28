#include "config.h"
#include "main.h"
#include "timer.h"
#include "mcl.h"

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

// pros::Task rumbleTask([]() {
//     while (true) {
//         if(!lock) master.rumble(".");
//         flush_logs();
//         pros::delay(1500); // Check every 100ms
//     }
// });

void opcontrol() {

    while (true) {

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            driveMode = (driveMode == DriveMode::TANK) ? DriveMode::ARCADE : DriveMode::TANK;
            master.rumble("."); // Short vibration to indicate mode change
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            autonomous();
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

        if(!shift1 && !shift2) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move_voltage(12000);
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

        else if(shift2) {
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                hood_up = !hood_up;
                hood_piston.set_value(hood_up);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                loading = !loading;
                loader_piston.set_value(loading);
            }
        }

        if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(0);
        }

        pros::delay(10);
    }
}