#pragma once

#include "odom.h"
#include "misc.h"
#include "api.h"

class MCLDistance {
    double data = -1;
    bool valid = false;
    bool enabled = true;
public:
    pros::Distance distance_sensor;
    double offset_x, offset_y;
    Orientation orientation;
    MCLDistance(int32_t port, double offset_x, double offset_y, Orientation orientation) : 
        distance_sensor(port), offset_x(offset_x), offset_y(offset_y), orientation(orientation) {}
    void set_enabled(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    double get_reading() { return data; }
};

struct Particle {
    Point position; 
    double weight; 
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