#pragma once

#include "pros/motors.h"
#include "miku-api.h"
#include <memory>

namespace miku {

class AbstractMotor : public pros::Motor {
      public:
      AbstractMotor(std::int8_t port, pros::v5::MotorGears gearset = pros::v5::MotorGears::blue, 
                    pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees);
      virtual double get_filtered_velocity();
      virtual ~AbstractMotor() = default;

      protected:

      int ticks_per_rev;

      double prev_ticks = 0;
      uint32_t last_time = 0;

      double sma_filter_size = 2;
      double median_filter_size = 3;
      double accel_filter_size = 10;
      std::vector<double> prev_raw_velocities;
      std::vector<double> prev_filtered_velocities;
      std::vector<double> prev_accels;
      double prev_estimated_velocity = 0;

};

class MotorController {
protected:
    bool velocity_control_enabled = false;
    PID velocity_pid;
    LookupTable voltage_lut;
    double max_voltage = 12000;

public:
    MotorController(PIDGains pid_gains, LookupTable voltage_lookup_table);
    virtual void move_velocity(double velocity);
};

class Motor : public AbstractMotor, public MotorController {

public:
Motor(std::int8_t port, pros::v5::MotorGears gearset = pros::v5::MotorGears::blue, 
      pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees,
      LookupTable voltage_lookup_table = LookupTable(), PIDGains pid_gains = PIDGains());

void move_velocity(double velocity) override;

};

class MotorGroup : public MotorController {

private:
std::vector<AbstractMotor*> motors;

public:
MotorGroup() = default;
MotorGroup(const std::initializer_list<std::int8_t> ports, 
           const pros::v5::MotorGears gearset = pros::v5::MotorGears::blue,
           const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees,
           LookupTable voltage_lookup_table = LookupTable(), PIDGains pid_gains = PIDGains());

void move_velocity(double velocity) override;

// Uses get_filtered_velocity() to return the average velocity of the group
double get_average_velocity();

double get_average_position();

void move(int voltage);
void move_voltage(int32_t voltage);
void tare_position(void) const;
void set_brake_mode(pros::motor_brake_mode_e mode);
double get_average_velocity();
std::vector<int> get_temperature_all() const;

};

} // namespace miku