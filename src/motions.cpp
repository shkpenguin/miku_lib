#include "time.h"
#include "system.h"
#include "config.h"
#include "motions.h"
#include "geometry.h"
#include "pid.h"
#include "lut.h"

void move_motors(double l, double r) {
    left_motors.move_voltage(l);
    right_motors.move_voltage(r);
}

void stop_motors() {
    left_motors.move_voltage(0);
    right_motors.move_voltage(0);
    pros::delay(10);
}

void set_drive_brake(pros::motor_brake_mode_e mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}