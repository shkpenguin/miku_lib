#pragma once

#include "api.h"
#include "miku-api.h"

#define WHEEL_DIAMETER 3.25
#define GEAR_RATIO 0.75
#define TRACK_WIDTH 12.0
#define DELTA_TIME 10
#define DEFAULT_AUTONOMOUS_BRAKE_MODE pros::E_MOTOR_BRAKE_BRAKE

#define LOGGING_ENABLED 1

extern pros::Controller master;

extern miku::MotorGroup left_motors;
extern miku::MotorGroup right_motors;
extern miku::Chassis miku;

extern miku::Motor left_front;
extern miku::Motor left_middle;
extern miku::Motor left_back;
extern miku::Motor right_front;
extern miku::Motor right_middle;
extern miku::Motor right_back;

extern miku::Motor bottom_intake;
extern miku::Motor top_intake;
extern miku::MotorGroup intake;

extern miku::Pneumatic hood_piston;
extern miku::Pneumatic lock_piston;
extern miku::Pneumatic loader_piston;
extern miku::Pneumatic descore_piston;

extern miku::Distance front_distance;
extern miku::Distance back_distance;
extern miku::Distance left_distance;
extern miku::Distance right_distance;

extern pros::Imu imu;

extern miku::Optical optical;

struct Gains;
struct RangeExit;

extern Gains turn_gains;
extern Gains drive_gains;

extern RangeExit drive_small_exit;
extern RangeExit drive_large_exit;
extern RangeExit turn_small_exit;
extern RangeExit turn_large_exit;

extern PatienceExit turn_patience_exit;
extern PatienceExit drive_patience_exit;