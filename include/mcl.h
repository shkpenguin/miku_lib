#pragma once

#include "miku-api.h"
#include <fstream>

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

extern MCLDistance back;
extern MCLDistance left;
extern MCLDistance right;
extern MCLDistance front;

extern std::ofstream file;

extern std::vector<std::reference_wrapper<MCLDistance>> sensors;

struct Particle {
    Point position; 
    double weight; 
    std::vector<double> sensor_readings;

    Particle() : position(0,0), weight(1.0) { sensor_readings.resize(sensors.size(), 0.0); }
};

void set_all_sensors(bool enabled);

double get_expected_reading(Pose particle_pose, Pose offset);
void log_mcl();
void flush_logs();
Pose get_pose_estimate();
void initialize_pose(Pose robot_pose);
void update_particles();
void resample_particles();