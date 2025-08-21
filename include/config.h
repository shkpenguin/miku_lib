#pragma once
#include "api.h"

#define WHEEL_DIAMETER 2.75
#define DT_GEAR_RATIO 1.0

extern pros::Controller master;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern pros::Motor left_front;
extern pros::Motor left_middle;
extern pros::Motor left_back;
extern pros::Motor right_front;
extern pros::Motor right_middle;
extern pros::Motor right_back;

extern pros::Motor bottom_intake;
extern pros::Motor top_intake;
extern pros::MotorGroup intake;

extern pros::adi::DigitalOut hood_piston;
extern pros::adi::DigitalOut lock_piston;
extern pros::adi::DigitalOut loader_piston;

extern pros::Distance back;

extern pros::Imu imu;