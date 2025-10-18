#pragma once

#include "miku-api.h"
#include <fstream>

enum class Orientation {
    LEFT,
    RIGHT,
    FRONT,
    BACK
};

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
void set_max_distance_error(double error);
void set_min_odom_noise(double noise);

void initialize_pose(Pose robot_pose);
void initialize_particles_uniform(Point center, double length);

void log_mcl();
void flush_logs();

double get_expected_reading(Point particle_position, double offset_x, double offset_y, 
    double cos_theta, double sin_theta, Orientation orientation);
Pose get_pose_estimate();
void update_particles();
void resample_particles();