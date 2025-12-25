#pragma once

#define NUM_PARTICLES 500

#include "miku/devices/distance.hpp"
#include "miku/geometry.hpp"
#include <fstream>

extern std::ofstream file;

enum WallID {
    LEFT_WALL,
    RIGHT_WALL,
    TOP_WALL,
    BOTTOM_WALL,
    NOT_IN_FIELD,
    BAD_INTERSECT
};

struct WallEstimate {
    WallID wall_id;
    float distance;
};

struct Particle {
    Point position; 
    float weight; 
    std::vector<WallEstimate> sensor_readings;

    Particle() : position(0,0), weight(1.0) {}
};

class ParticleFilter {
    public:
    float max_error = 8.0; // inches
    float min_odom_noise = 0.05; // inches
    std::vector<Particle> particles = std::vector<Particle>(NUM_PARTICLES);

    ParticleFilter(std::vector<std::shared_ptr<miku::Distance>> sensors);
    void set_max_distance_error(float error);
    void set_min_odom_noise(float noise);
    void set_max_sensor_stdev(float stdev);
    void set_particles_point(Point center);
    void set_particles_uniform(Point center, float length);
    void update_previous_belief(Pose robot_speed);
    void update_particle_weights();
    Point get_current_belief();
    void resample_particles();

    std::vector<std::shared_ptr<miku::Distance>> distance_sensors;

};