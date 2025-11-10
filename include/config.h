#pragma once

#include "api.h"
#include "miku/devices/chassis.h"
#include "miku/devices/motor.h"
#include "miku/devices/controller.h"
#include "miku/devices/pneumatic.h"
#include "miku/devices/distance.h"
#include "miku/devices/optical.h"

#include "miku/exit.h"

#define WHEEL_DIAMETER 3.25
#define GEAR_RATIO 0.75
#define TRACK_WIDTH 12.0
#define DELTA_TIME 10
#define DEFAULT_AUTONOMOUS_BRAKE_MODE pros::E_MOTOR_BRAKE_BRAKE

#define LOGGING_ENABLED 1

extern miku::Controller master;

extern miku::MotorGroup left_motors;
extern miku::MotorGroup right_motors;
extern miku::Chassis Miku;

extern miku::Motor left_front;
extern miku::Motor left_middle;
extern miku::Motor left_back;
extern miku::Motor right_front;
extern miku::Motor right_middle;
extern miku::Motor right_back;

extern miku::Motor intake_bottom;
extern miku::Motor intake_top;

extern miku::Pneumatic loader_piston;
extern miku::Pneumatic lock_piston;
extern miku::Pneumatic middle_piston;
extern miku::Pneumatic descore_piston;

extern miku::Distance front_distance;
extern miku::Distance back_distance;
extern miku::Distance left_distance;
extern miku::Distance right_distance;

extern pros::Imu imu;

extern miku::Optical optical;

extern PIDGains turn_gains;
extern PIDGains drive_gains;

extern RangeExit drive_small_exit;
extern RangeExit drive_large_exit;
extern RangeExit turn_small_exit;
extern RangeExit turn_large_exit;

extern PatienceExit turn_patience_exit;
extern PatienceExit drive_patience_exit;