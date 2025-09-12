#define NUM_PARTICLES 500
#define INIT_STDEV 1.0
#define SENSOR_STDEV 0.75

// #define LATERAL_STDEV 0.2

#include "api.h"
#include "odom.h"
#include "util.h"
#include "mcl.h"
#include "config.h"
#include <random>
#include <cmath>
#include <vector>
#include <chrono>
#include <fstream>
#include <climits>

MCLDistance back(10, Pose(3.0, -6.25, M_PI));
MCLDistance left(2, Pose(-5.0, 1.375, M_PI/2));
MCLDistance right(9, Pose(5.0, 1.375, -M_PI/2));
MCLDistance front(8, Pose(3.25, 8.25, 0.0));

std::vector<MCLDistance*> sensors = {&front, &back, &left, &right};

void MCLDistance::update_reading() {

    if(!enabled) valid = false;

    bool visible = distance_sensor.get_object_size() > 50 || distance_sensor.get_distance() < 100;

    bool within_range = distance_sensor.get_distance() < 2000;

    if(visible && within_range) {
        data = distance_sensor.get_distance() / 25.4;
        valid = true;
    } else {
        data = -1;
        valid = false;
    }

}

// MCLDistance::set_value(bool enable) { enabled = enable; }

inline void set_all_sensors(bool enabled) {
    for(auto sensor : sensors) {
        sensor->set_enabled(enabled);
    }
}

double get_expected_reading(Pose particle_pose, Pose offset) {
    double robot_theta = getPose(true).theta;

    double cos_theta = cos(robot_theta);
    double sin_theta = sin(robot_theta);

    double sensor_x = particle_pose.x + offset.x * sin_theta + offset.y * cos_theta;
    double sensor_y = particle_pose.y - offset.x * cos_theta + offset.y * sin_theta;

    double dx = cos(robot_theta + offset.theta);
    double dy = sin(robot_theta + offset.theta);

    double tMin = std::numeric_limits<double>::infinity();

    // Check vertical walls
    if (fabs(dx) > 1e-6) {
        double t1 = (-HALF_FIELD - sensor_x) / dx;
        if (t1 > 0) {
            double y1 = sensor_y + t1 * dy;
            if (y1 >= -HALF_FIELD && y1 <= HALF_FIELD)
                tMin = std::min(tMin, t1);
        }
        double t2 = (HALF_FIELD - sensor_x) / dx;
        if (t2 > 0) {
            double y2 = sensor_y + t2 * dy;
            if (y2 >= -HALF_FIELD && y2 <= HALF_FIELD)
                tMin = std::min(tMin, t2);
        }
    }

    // Check horizontal walls
    if (fabs(dy) > 1e-6) {
        double t3 = (-HALF_FIELD - sensor_y) / dy;
        if (t3 > 0) {
            double x3 = sensor_x + t3 * dx;
            if (x3 >= -HALF_FIELD && x3 <= HALF_FIELD)
                tMin = std::min(tMin, t3);
        }
        double t4 = (HALF_FIELD - sensor_y) / dy;
        if (t4 > 0) {
            double x4 = sensor_x + t4 * dx;
            if (x4 >= -HALF_FIELD && x4 <= HALF_FIELD)
                tMin = std::min(tMin, t4);
        }
    }

    if (!std::isfinite(tMin)) return -2 ; // no valid hit

    double x_intersect = sensor_x + tMin * dx;
    double y_intersect = sensor_y + tMin * dy;

    // Deadzone check (example for top/bottom walls)
    if (fabs(y_intersect - HALF_FIELD) < 1.0 || fabs(y_intersect + HALF_FIELD) < 1.0) {
        if ((x_intersect >= -54.0 && x_intersect <= -42.0) ||
            (x_intersect >=  42.0 && x_intersect <=  54.0)) {
            return -1;
        }
    }

    return tMin; // distance along ray

}

std::vector<Particle> particles(NUM_PARTICLES);
std::mt19937_64 rng(std::random_device{}());

std::ofstream file;
std::ostringstream log_buffer;

void log_mcl() {

    Pose robot_pose = getPose();

    log_buffer << NUM_PARTICLES << "," << robot_pose.x << "," << robot_pose.y << "," << robot_pose.theta << ",";
    log_buffer << left.distance_sensor.get_distance() / 25.4 << "," 
               << right.distance_sensor.get_distance() / 25.4 << "," 
               << back.distance_sensor.get_distance() / 25.4 << ",";
    log_buffer << left.distance_sensor.get_object_size() << "," 
               << right.distance_sensor.get_object_size() << "," 
               << back.distance_sensor.get_object_size() << ",";

    for(size_t i = 0; i < particles.size(); ++i) {
        log_buffer << particles[i].pose.x << "," 
                << particles[i].pose.y << "," 
                << particles[i].weight;
        if(i != particles.size()-1) log_buffer << ","; // no trailing comma
    }
    log_buffer << "\n";
    
}

void flush_logs() {
    file << log_buffer.str();
    file.flush();
    log_buffer.str("");
    log_buffer.clear();
}

std::normal_distribution<double> init(0, INIT_STDEV);

// DOES NOT FIX THETA
Pose get_pose_estimate() {
    double x = 0.0;
    double y = 0.0;
    double total_weight = 0.0;

    for(const auto& particle : particles) {
        x += particle.pose.x * particle.weight;
        y += particle.pose.y * particle.weight;
        total_weight += particle.weight;
    }

    if(total_weight > 0) {
        x /= total_weight;
        y /= total_weight;
    }

    return Pose(x, y, 0.0);
}

void initialize_mcl() {

    file.open("log.txt");

    // if(!file.is_open()) {
    //     master.set_text(0, 0, "Failed to open log file");
    //     return;
    // }

    Pose robot_pose = getPose();

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        double rand_x = init(rng);
        double rand_y = init(rng);
        particles[i].pose = Pose(clamp_field(robot_pose.x + rand_x), clamp_field(robot_pose.y + rand_y), robot_pose.theta);
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

    // log_mcl();
}

#define MAX_ERROR 6.0
// #define MAX_LATERAL_ERROR 6.0

// std::normal_distribution<double> lateral_noise(0, LATERAL_STDEV);
static std::normal_distribution<double> odom_noise;

void update_particles() {

    double velocity = sqrt(getSpeed().x * getSpeed().x + getSpeed().y * getSpeed().y);
    odom_noise = std::normal_distribution<double>(0, velocity / 4);

    Pose robot_speed = getSpeed();
    double robot_theta = getPose(true).theta;
    double sin_theta = sin(robot_theta);
    double cos_theta = cos(robot_theta);

    for(auto &sensor : sensors) {
        sensor->update_reading();
    }

    double total_weight = 0.0;

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].pose.x = clamp_field(
            particles[i].pose.x + 
            robot_speed.x + 
            odom_noise(rng));
        particles[i].pose.y = clamp_field(
            particles[i].pose.y + 
            robot_speed.y + 
            odom_noise(rng));

        double weight = 1.0;

        for(auto &sensor : sensors) {
            if(!sensor->get_enabled() || !sensor->get_valid()) continue;
            double expected = get_expected_reading(particles[i].pose, sensor->offset);
            if(expected == -1) continue;
            if(expected == -2) {
                weight *= 0.0;
                break;
            }
            double reading = sensor->get_reading();
            double dev = reading - expected;
            if(fabs(dev) > MAX_ERROR) {
                weight *= 0.0;
                break;
            }
            else weight *= std::exp(-(dev * dev) / (2 * SENSOR_STDEV * SENSOR_STDEV));
        }

        particles[i].weight = weight;
        total_weight += weight;
    }

    if(total_weight == 0.0) {
        double weight = 1.0 / NUM_PARTICLES;
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight = weight;
        }
    } else {
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight /= total_weight;
        }
    }

}

void resample_particles() {
    std::vector<Particle> newParticles(NUM_PARTICLES);
    std::uniform_real_distribution<double> dist(0.0, 1.0 / NUM_PARTICLES);

    double r = dist(rng);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; m++) {
        double U = r + (double)m / NUM_PARTICLES;
        while (U > c && i < NUM_PARTICLES-1) {
            i++;
            c += particles[i].weight;
        }
        newParticles[m] = particles[i];  // copy particle
    }

    for (auto &p : newParticles) {
        p.weight = 1.0 / NUM_PARTICLES;
    }
    particles = newParticles;
    // log_mcl();
}