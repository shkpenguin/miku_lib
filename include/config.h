#pragma once

#include "api.h"
#include "motor.h"

#define WHEEL_DIAMETER 3.25
#define GEAR_RATIO 0.75
#define TRACK_WIDTH 12.0
#define DELTA_TIME 10

#define LOGGING_ENABLED 1

extern pros::Controller master;

extern miku::MotorGroup left_motors;
extern miku::MotorGroup right_motors;

extern miku::Motor left_front;
extern miku::Motor left_middle;
extern miku::Motor left_back;
extern miku::Motor right_front;
extern miku::Motor right_middle;
extern miku::Motor right_back;

extern miku::Motor bottom_intake;
extern miku::Motor top_intake;
extern miku::MotorGroup intake;

extern pros::adi::DigitalOut hood_piston;
extern pros::adi::DigitalOut lock_piston;
extern pros::adi::DigitalOut loader_piston;
extern pros::adi::DigitalOut descore_piston;

extern pros::Imu imu;

struct Gains;
struct ExitCondition;

extern Gains drive_gains;

extern ExitCondition drive_small_exit;
extern ExitCondition drive_large_exit;
extern ExitCondition turn_small_exit;
extern ExitCondition turn_large_exit;