#pragma once

#include "odom.h"
#include "api.h"

class MCLDistance {
    double data = -1;

public:
    pros::Distance distance_sensor;
    Pose offset;
    MCLDistance(int32_t port, Pose offset) : distance_sensor(port), offset(offset) {}
    bool update_reading();
    double get_reading();
};

extern MCLDistance back;
extern MCLDistance left;
extern MCLDistance right;

double get_expected_reading(Pose particle_pose, Pose offset);
void log_mcl();
void flush_logs();
Pose get_pose_estimate();
void initialize_particles();
void update_particles();
void resampleParticles();