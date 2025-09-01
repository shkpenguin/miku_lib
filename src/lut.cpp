#include "api.h"
#include "misc.h"
#include "config.h"
#include <numeric>

std::vector<std::pair<double, double>> lookup = {
    {-100.0, -12000},
    {-93.0, -11000}, 
    {-85.0, -10000}, 
    {-79.0, -9000},
    {-73.0, -8500},
    {-68.5, -8000}, 
    {-65.0, -7500},
    {-60.5, -7000}, 
    {-55.5, -6500},
    {-51.5, -6000},
    {-46.7, -5500},
    {-42.3, -5000},
    {-36.8, -4500},
    {-32.5, -4000}, 
    {-27.8, -3500},
    {-23.5, -3000}, 
    {-17.4, -2500},
    {-12.8, -2000}, 
    {-8.5, -1500}, // 
    {-4.0, -1000},
    {0, -500},
    {0, 0},
    {0, 500},
    {3.0, 1000}, 
    {8.3, 1500}, // 
    {12.4, 2000}, 
    {17.5, 2500},
    {23.8, 3000},
    {28.0, 3500},
    {32.7, 4000},
    {37.5, 4500},
    {41.8, 5000}, 
    {46.5, 5500},
    {52.0, 6000}, 
    {56.0, 6500},
    {61.0, 7000}, 
    {65.5, 7500},
    {70.0, 8000},
    {73.5, 8500},
    {79.0, 9000}, 
    {88.0, 10000}, 
    {93.0, 11000}, 
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