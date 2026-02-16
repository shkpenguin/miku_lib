#pragma once

#include "api.h"
#include "miku/devices/chassis.hpp"
#include "miku/devices/motor.hpp"
#include "miku/devices/controller.hpp"
#include "miku/devices/pneumatic.hpp"
#include "miku/devices/distance.hpp"
#include "miku/devices/optical.hpp"
#include "miku/devices/intake.hpp"
#include "miku/mcl.hpp"

#include "miku/exit.hpp"

#define WHEEL_DIAMETER 3.25f
#define WHEEL_CIRC (WHEEL_DIAMETER * M_PI)
#define GEAR_RATIO 0.75f
#define TRACK_WIDTH 10.75f
#define MAX_RPM 660.0f
#define MAX_VEL 84.233953f // in/s (WHEEL_CIRC * MAX_RPM * GEAR_RATIO / 60)
#define MAX_ANG_VEL 16.8467906f // rad/s (2 * MAX_VEL / TRACK_WIDTH)
#define DELTA_TIME 10
#define DEFAULT_AUTONOMOUS_BRAKE_MODE pros::E_MOTOR_BRAKE_BRAKE

extern miku::Controller master;

extern miku::MotorGroup left_motors;
extern miku::MotorGroup right_motors;

extern pros::Imu imu;

extern miku::Distance &front_distance;
extern miku::Distance &back_distance;
extern miku::Distance &left_distance;
extern miku::Distance &right_distance;

extern ParticleFilter mcl;

extern miku::Chassis Miku;

extern miku::Motor intake_bottom;
extern miku::Motor intake_middle;
extern miku::Motor intake_top;

extern miku::Intake intake;

extern miku::Pneumatic loader_piston;
extern miku::Pneumatic lock_piston;
extern miku::Pneumatic middle_piston;
extern miku::Pneumatic descore_piston;

// extern miku::Optical intake_optical;
extern miku::Optical floor_optical;

extern PIDGains turn_gains;
extern PIDGains drive_gains;

extern RangeExit drive_small_exit;
extern RangeExit drive_large_exit;
extern RangeExit turn_small_exit;
extern RangeExit turn_large_exit;

extern PatienceExit turn_patience_exit;
extern PatienceExit drive_quick_exit;
extern PatienceExit drive_slow_exit;