#pragma once

#include "pros/motors.h"
#include "pros/abstract_motor.hpp"
#include "config.h"

namespace miku {

class Motor : public pros::Motor {

int ticks_per_rev;

double prev_ticks = 0;
u_int32_t last_time = 0;

double sma_filter_size = 3;
double median_filter_size = 7;
double accel_filter_size = 20;
std::vector<double> prev_raw_velocities;
std::vector<double> prev_filtered_velocities;
std::vector<double> prev_accels;
double prev_estimated_velocity = 0;

public:
double get_estimated_velocity();

Motor(std::int8_t port, pros::v5::MotorGears gearset = pros::v5::MotorGears::blue, 
      pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees) : 
      pros::Motor(port, gearset, encoder_units) {
        if(gearset == pros::v5::MotorGears::red) {
            ticks_per_rev = 1800;
        } else if(gearset == pros::v5::MotorGears::green) {
            ticks_per_rev = 900;
        } else if(gearset == pros::v5::MotorGears::blue) {
            ticks_per_rev = 300;
        } else {
            std::cerr << "Invalid gearset for motor on port " << static_cast<int>(port) << std::endl;
        }
    }
};

class MotorGroup {
std::vector<Motor*> motors;
public:
MotorGroup(const std::initializer_list<std::int8_t> ports);

std::int32_t move(int voltage);

std::int32_t move_voltage(int voltage);

std::vector<double> get_estimated_velocities();

// Uses get_estimated_velocity() to return the average velocity of the group
double get_average_velocity();

std::int32_t tare_position_all(void) const;

std::int32_t set_brake_mode_all(pros::motor_brake_mode_e mode);

};

} // namespace miku