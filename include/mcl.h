#pragma once

#include "odom.h"
#include "api.h"

class MCLDistance {
    double data = -1;
    bool valid = false;
    bool enabled = true;
public:
    pros::Distance distance_sensor;
    Pose offset;
    MCLDistance(int32_t port, Pose offset) : distance_sensor(port), offset(offset) {}
    void set_value(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    double get_reading() { return data; }
};

extern MCLDistance back;
extern MCLDistance left;
extern MCLDistance right;
extern MCLDistance front;

void set_all_sensors(bool enabled);

double get_expected_reading(Pose particle_pose, Pose offset);
void log_mcl();
void flush_logs();
Pose get_pose_estimate();
void initialize_mcl();
void update_particles();
void resample_particles();