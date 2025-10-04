#pragma once

#include <vector>
#include "motions.h"
#include "util.h"
#include "odom.h"
#include "config.h"
#include "timer.h"

void move_motors(double l, double r);

void stop_motors();

void set_drive_brake(pros::motor_brake_mode_e mode);

// Motion control functions
void wait_until_done();
void wait_until_within(Point target, double threshold);
void request_motion_start();
void end_motion();
void cancel_motion();
void cancel_all_motions();

struct TurnParams {
    bool reverse = false;
    bool async = false;
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 0;
};

struct SwingParams {
    bool reverse = false;
    bool async = false;
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 0;
};

struct MovePointParams {
    bool reverse = false;
    bool async = false;
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 1000;
};

struct MovePoseParams {
    bool reverse = false;
    bool async = false;
    double cutoff = -1.0;
    double max_vel = 100;
    double min_vel = 0;
    double distance_weight = 4.0;
    double angular_weight = 1.0; 
    double end_cutoff = 6.0;
};

struct RamseteParams {
    bool reverse = false;
    bool async = false;
    double cutoff = -1.0;
    double max_vel = 100;
    double min_vel = 0;
    double angular_weight = 0.003;
    double end_cutoff = 6.0;
};

void turn_heading(double target, double timeout, TurnParams params = TurnParams(false, false, -1.0, 12000, 0));
void turn_point(Point target, double timeout, TurnParams params = TurnParams(false, false, -1.0, 12000, 0));

void swing_heading(double target, Side locked_side, double timeout, SwingParams params = SwingParams(false, false, -1.0, 12000, 0));
void swing_point(Point target, Side locked_side, double timeout, SwingParams params = SwingParams(false, false, -1.0, 12000, 0));

void move_point(Point target, double timeout, MovePointParams params = MovePointParams(false, false, -1.0, 12000, 1000));
void move_pose(Pose target, double timeout, MovePoseParams params = MovePoseParams(false, false, -1.0, 100, 0.0, 1.0, 6.0));
void move_time(double volts, double timeout);

void ramsete(std::vector<Waypoint> waypoints, double timeout, RamseteParams params = RamseteParams(false, false, -1.0, 100, 0.0, 0.003, 6.0));