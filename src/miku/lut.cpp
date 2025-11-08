#include "lut.h"
#include "config.h"
#include <numeric>

double LookupTable::get_value(double key) {
    // use binary search to find least voltage below desired velocity
    int low = 0;
    int high = table.size() - 1;
    while (low < high) {
        int mid = (low + high + 1) / 2;
        if (table[mid].first <= key) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    // use linear interpolation
    if (low == table.size() - 1) {
        return table[low].second;
    }
    double x0 = table[low].first;
    double y0 = table[low].second;
    double x1 = table[low + 1].first;
    double y1 = table[low + 1].second;
    return y0 + (y1 - y0) * (key - x0) / (x1 - x0);
}

void tune_lut_drive() {

    double drive_voltage = 0;

    while(true) {

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            drive_voltage += 500;
        } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            drive_voltage -= 500;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) drive_voltage *= -1;

        double left_avg = left_motors.get_average_velocity();
        double right_avg = right_motors.get_average_velocity();

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            left_motors.move_voltage(drive_voltage);
            right_motors.move_voltage(drive_voltage);
        } else {
            left_motors.move_voltage(0);
            right_motors.move_voltage(0);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            master.set_text(0, 0, "velocity: " + std::to_string((left_avg + right_avg) / (2)));
        } else {
            master.set_text(0, 0, "voltage: " + std::to_string(drive_voltage));
        }

        pros::delay(20);

    }
}

void tune_lut_intake() {

    double intake_voltage = 0;

    while(true) {

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intake_voltage += 500;
        } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            intake_voltage -= 500;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) intake_voltage *= -1;

        double velocity = intake.get_average_velocity();

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake.move_voltage(intake_voltage);
        } else {
            intake.move_voltage(0);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            master.set_text(0, 0, "velocity: " + std::to_string(velocity / 6));
        } else {
            master.set_text(0, 0, "voltage: " + std::to_string(intake_voltage));
        }

        pros::delay(20);

    }
}