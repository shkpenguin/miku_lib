#pragma once

#include "miku-api.h"
#include <fstream>

extern std::ofstream file;

extern std::vector<std::reference_wrapper<miku::Distance>> sensors;

struct Particle {
    Point position; 
    double weight; 
    std::vector<double> sensor_readings;

    Particle() : position(0,0), weight(1.0) { sensor_readings.resize(sensors.size(), 0.0); }
};

void set_all_sensors(bool enabled);
void set_max_distance_error(double error);
void set_min_odom_noise(double noise);
void set_max_sensor_stdev(double stdev);

void initialize_particles_point(Point center);
void initialize_particles_uniform(Point center, double length);

void log_mcl();
void flush_logs();

double get_expected_reading(Point particle_position, double offset_x, double offset_y, 
    double cos_theta, double sin_theta, Orientation orientation);
Point get_position_estimate();
void update_particles();
void resample_particles();