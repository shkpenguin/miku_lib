#include "api.h"
#include "misc.h"
#include "config.h"
#include <numeric>

std::vector<std::pair<double, double>> lookup = {
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
    {-9.5, -1500}, // 
    {-5.8, -1000},
    {0.0, -500},
    {0.0, 0},
    {0.0, 500},
    {6.2, 1000}, 
    {10.0, 1500}, // 
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

double voltage_lookup(double velocity) {
    // use binary search to find least voltage below desired velocity
    int low = 0;
    int high = lookup.size() - 1;
    while (low < high) {
        int mid = (low + high + 1) / 2;
        if (lookup[mid].first <= velocity) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    // use linear interpolation
    if (low == lookup.size() - 1) {
        return lookup[low].second;
    }
    double x0 = lookup[low].first;
    double y0 = lookup[low].second;
    double x1 = lookup[low + 1].first;
    double y1 = lookup[low + 1].second;
    return y0 + (y1 - y0) * (velocity - x0) / (x1 - x0);
}

void tune_lut() {

    double drive_voltage = 0;

    while(true) {

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            drive_voltage += 500;
        } else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            drive_voltage -= 500;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) drive_voltage *= -1;

        std::vector<double> left_vel = left_motors.get_actual_velocity_all();
        std::vector<double> right_vel = right_motors.get_actual_velocity_all();
        double left_avg = std::accumulate(left_vel.begin(), left_vel.end(), 0.0) / left_vel.size();
        double right_avg = std::accumulate(right_vel.begin(), right_vel.end(), 0.0) / right_vel.size();

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            left_motors.move_voltage(drive_voltage);
            right_motors.move_voltage(drive_voltage);
        } else {
            left_motors.move_voltage(0);
            right_motors.move_voltage(0);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            master.set_text(0, 0, "velocity: " + std::to_string((left_avg + right_avg) / (2 * 2)));
        } else {
            master.set_text(0, 0, "voltage: " + std::to_string(drive_voltage));
        }

        pros::delay(20);

    }
}