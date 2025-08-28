#pragma once

#include <vector>
#include "motions.h"
#include "util.h"
#include "odom.h"
#include "config.h"
#include "timer.h"

void move_motors(double l, double r, bool reversed = false);

void stop_motors();

void set_drive_brake(pros::motor_brake_mode_e mode);

// Motion control functions
void wait_until_done();
void request_motion_start();
void end_motion();
void cancel_motion();
void cancel_all_motions();

void turn_heading(double target, bool reverse, double timeout, bool async = false, double cutoff = -1.0);
void turn_point(Point target, bool reverse, double timeout, bool async = false, double cutoff = -1.0);

void swing_heading(double target, Side locked_side, bool reverse, double timeout, bool async = false, double cutoff = -1.0);
void swing_point(Point target, Side locked_side, bool reverse, double timeout, bool async = false, double cutoff = -1.0);

void move_point(Point target, bool reverse, double timeout, bool async = false, double cutoff = -1.0);

void ramsete(std::vector<Waypoint> waypoints, double b, double zeta, double time_multi,
             double cutoff, double kP, double kD, bool reversed, double t_limit,
             bool async = false);