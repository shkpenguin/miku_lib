#pragma once

#include "api.h"
#include "miku/pid.hpp"

namespace miku {
class Controller {
public:
  void rumble(const char *pattern) {}
};
class Motor {
public:
  void move(int voltage) {}
};
class MotorGroup {
public:
  void move(int voltage) {}
};
class Pneumatic {
public:
  void set_value(bool value) {}
};
class Distance {
public:
  int get() { return 0; }
};
class Optical {
public:
  double get_hue() { return 0.0; }
};
class Intake {
public:
  void set(int speed, int voltage) {}
};
class Chassis {
public:
};
}

extern miku::Controller master;
extern miku::MotorGroup left_motors;
extern miku::MotorGroup right_motors;
extern pros::Imu imu;

#define WHEEL_DIAMETER 3.25f
#define WHEEL_CIRC (WHEEL_DIAMETER * M_PI)
#define GEAR_RATIO 0.75f
#define TRACK_WIDTH 10.75f
#define MAX_RPM 660.0f
#define DELTA_TIME 10

struct RangeExit {};
struct PatienceExit {};

extern PIDGains turn_gains;
extern PIDGains drive_gains;
