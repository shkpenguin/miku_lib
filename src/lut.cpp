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
    {-620.0, -12000},
    {-582.0, -11000},
    {-528.0, -10000},
    {-471.0, -9000},
    {-408.0, -8000},
    {-348.0, -7000},
    {-294.0, -6000},
    {-228.0, -5000},
    {-180.0, -4000},
    {-120.0, -3000},
    {-60.0, -2000},
    {0.0, -1500},
    {0.0, 0},
    {0.0, 1500},
    {60.0, 2000},
    {120.0, 3000},
    {180.0, 4000},
    {222.0, 5000},
    {288.0, 6000},
    {348.0, 7000},
    {399.0, 8000},
    {468.0, 9000},
    {519.0, 10000},
    {579.0, 11000},
    {620.0, 12000}
};

std::vector<std::pair<double, double>> turn_kp_lookup = {
    {0, 300},
    {10, 300},
    {20, 170},
    {30, 150},
    {40, 130},
    {50, 115},
    {60, 100},
    {70, 100},
    {80, 95},
    {180, 80}
};

std::vector<std::pair<double, double>> turn_kd_lookup = {
    {0, 0},
    {10, 0},
    {20, 800},
    {30, 1100},
    {40, 800},
    {50, 800},
    {60, 600},
    {70, 800},
    {80, 600},
    {180, 400}
};

LookupTable drive_lut(drive_table);
LookupTable intake_lut(intake_table);
LookupTable turn_kp_lut(turn_kp_lookup);
LookupTable turn_kd_lut(turn_kd_lookup);

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