#pragma once

#include "api.h"
#include "misc.h"

#define WHEEL_DIAMETER 2.75
#define DT_GEAR_RATIO 1.0
#define TRACK_WIDTH 12.5

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
extern pros::adi::DigitalOut descore_piston;

// extern pros::Distance back;

extern pros::Imu imu;

extern Gains drive_gains;
extern Gains turn_gains;
extern Gains angular_gains;

extern ExitCondition drive_small_exit;
extern ExitCondition drive_large_exit;
extern ExitCondition turn_small_exit;
extern ExitCondition turn_large_exit;