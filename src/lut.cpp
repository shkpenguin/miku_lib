#include "api.h"
#include "lut.h"
#include "config.h"
#include <numeric>

std::vector<std::pair<double, double>> drive_table = {
    {-100.0, -12000},
    {-99.0, -11000}, 
    {-91.0, -10000}, 
    {-83.0, -9000},
    {-78.0, -8500},
    {-73.0, -8000}, 
    {-68.0, -7500},
    {-64.0, -7000}, 
    {-59.0, -6500},
    {-54.0, -6000},
    {-48.5, -5500},
    {-44.0, -5000},
    {-40.0, -4500},
    {-35.0, -4000}, 
    {-30.0, -3500},
    {-25.0, -3000}, 
    {-18.5, -2500},
    {-14.2, -2000}, 
    {-9.5, -1500},
    {-5.8, -1000},
    {0.0, -500},
    {0.0, 0},
    {0.0, 500},
    {6.2, 1000}, 
    {10.0, 1500},
    {15.0, 2000}, 
    {20.0, 2500},
    {26.0, 3000},
    {30.0, 3500},
    {35.0, 4000},
    {40.0, 4500},
    {44.0, 5000}, 
    {49.5, 5500},
    {54.0, 6000}, 
    {59.5, 6500},
    {64.5, 7000}, 
    {68.5, 7500},
    {73.5, 8000},
    {78.0, 8500},
    {83.0, 9000}, 
    {92.0, 10000}, 
    {99.0, 11000}, 
    {100, 12000}
};

std::vector<std::pair<double, double>> intake_table = {
    {-100.0, -12000},
    {-97.0, -11000},
    {-88.0, -10000},
    {-78.5, -9000},
    {-68.0, -8000},
    {-58.0, -7000},
    {-49.0, -6000},
    {-38.0, -5000},
    {-30.0, -4000},
    {-20.0, -3000},
    {-10.0, -2000},
    {0.0, -1500},
    {0.0, 0},
    {0.0, 1500},
    {10.0, 2000},
    {20.0, 3000},
    {30.0, 4000},
    {37.0, 5000},
    {48.0, 6000},
    {58.0, 7000},
    {66.5, 8000},
    {78.0, 9000},
    {86.5, 10000},
    {96.5, 11000},
    {100.0, 12000}
};

LookupTable drive_lut(drive_table);
LookupTable intake_lut(intake_table);

double LookupTable::get_voltage(double velocity) {
    // use binary search to find least voltage below desired velocity
    int low = 0;
    int high = table.size() - 1;
    while (low < high) {
        int mid = (low + high + 1) / 2;
        if (table[mid].first <= velocity) {
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
    return y0 + (y1 - y0) * (velocity - x0) / (x1 - x0);
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