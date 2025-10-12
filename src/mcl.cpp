#define NUM_PARTICLES 1000

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

MCLDistance back(9, 3.5, -4.75, Orientation::BACK);
MCLDistance left(4, -5.2, 0, Orientation::LEFT);
MCLDistance right(8, 5.2, 0, Orientation::RIGHT);
MCLDistance front(7, -3.25, 8.25, Orientation::FRONT);

std::vector<std::reference_wrapper<MCLDistance>> sensors = {std::ref(front), std::ref(back), std::ref(left), std::ref(right)};

void MCLDistance::update_reading() {

    if(!enabled) valid = false;

    bool visible = distance_sensor.get_object_size() > 20 || distance_sensor.get_distance() < 100;

    bool within_range = distance_sensor.get_distance() < 2000;

    if(visible && within_range) {
        data = distance_sensor.get_distance() / 25.4;
        valid = true;
    } else {
        data = -1;
        valid = false;
    }

}

inline void set_all_sensors(bool enabled) {
    for(auto sensor : sensors) {
        sensor.get().set_enabled(enabled);
    }
}

double get_expected_reading(Point particle_position, double offset_x, double offset_y, double cos_theta, double sin_theta, Orientation orientation) {

    double sensor_x = particle_position.x + offset_x * sin_theta + offset_y * cos_theta;
    double sensor_y = particle_position.y - offset_x * cos_theta + offset_y * sin_theta;

    double dx, dy;

    switch (orientation) {
        case Orientation::FRONT:
            dx = cos_theta;
            dy = sin_theta;
            break;
        case Orientation::BACK:
            dx = -cos_theta;
            dy = -sin_theta;
            break;
        case Orientation::LEFT:
            dx = -sin_theta;
            dy = cos_theta;
            break;
        case Orientation::RIGHT:
            dx = sin_theta;
            dy = -cos_theta;
            break;
    }

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

    if (!std::isfinite(tMin)) return -1; // no valid hit

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

    Pose robot_pose = get_pose();

    log_buffer << NUM_PARTICLES << "," << robot_pose.x << "," << robot_pose.y << "," << robot_pose.theta << ",";
    log_buffer << left.distance_sensor.get_distance() / 25.4 << "," 
               << right.distance_sensor.get_distance() / 25.4 << "," 
               << front.distance_sensor.get_distance() / 25.4 << ","
               << back.distance_sensor.get_distance() / 25.4 << ",";
    log_buffer << left.distance_sensor.get_object_size() << "," 
               << right.distance_sensor.get_object_size() << "," 
                << front.distance_sensor.get_object_size() << ","
               << back.distance_sensor.get_object_size() << ",";

    for(size_t i = 0; i < particles.size(); ++i) {
        log_buffer << particles[i].position.x << "," 
                << particles[i].position.y << "," 
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

Pose get_pose_estimate() {
    double x = 0.0;
    double y = 0.0;
    double total_weight = 0.0;

    for(const auto& particle : particles) {
        x += particle.position.x * particle.weight;
        y += particle.position.y * particle.weight;
        total_weight += particle.weight;
    }

    if(total_weight > 0) {
        x /= total_weight;
        y /= total_weight;
    }

    return Pose(x, y, 0.0);
}

void initialize_pose(Pose robot_pose) {

    start_odom(robot_pose);

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = Point(robot_pose.x, robot_pose.y);
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

void initialize_particles_uniform(Point center, double length) {

    std::uniform_real_distribution<double> dist(length / -2, length / 2);

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = Point(
            clamp_field(center.x + dist(rng)), 
            clamp_field(center.y + dist(rng))
        );
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

#define MAX_ERROR 8.0
static std::normal_distribution<double> odom_noise;

void update_particles() {

    double velocity = sqrt(get_speed().x * get_speed().x + get_speed().y * get_speed().y);
    double odom_stdev = std::max(velocity / 4, 0.1);
    odom_noise = std::normal_distribution<double>(0, odom_stdev);

    Pose robot_speed = get_speed();
    double robot_theta = get_pose({.standard = true}).theta;
    double sin_theta = sin(robot_theta);
    double cos_theta = cos(robot_theta);

    std::vector<bool> valid_sensors(sensors.size(), true);

    // max error check
    for(size_t i = 0; i < sensors.size(); ++i) {
        double expected = get_expected_reading(
            Point(get_pose().x, get_pose().y), 
            sensors[i].get().offset_x, 
            sensors[i].get().offset_y, 
            cos_theta, 
            sin_theta, 
            sensors[i].get().orientation);
        if(fabs(expected - sensors[i].get().get_reading()) > MAX_ERROR) valid_sensors[i] = false;
    }

    // check for particles with invalid readings
    for(size_t i = 0; i < particles.size(); ++i) {
        for(size_t j = 0; j < sensors.size(); ++j) {
            particles[i].sensor_readings[j] = get_expected_reading(
                particles[i].position, 
                sensors[j].get().offset_x, 
                sensors[j].get().offset_y, 
                cos_theta, 
                sin_theta, 
                sensors[j].get().orientation);
            if(particles[i].sensor_readings[j] == -1) valid_sensors[j] = false;
        }
    }

    for(auto &sensor : sensors) {
        sensor.get().update_reading();
    }

    double total_weight = 0.0;

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position.x = clamp_field(
            particles[i].position.x + 
            robot_speed.x + 
            odom_noise(rng));
        particles[i].position.y = clamp_field(
            particles[i].position.y + 
            robot_speed.y + 
            odom_noise(rng));

        double weight = 1.0;

        for(int j = 0; j < sensors.size(); ++j) {
            if(!valid_sensors[j] || !sensors[j].get().get_enabled() || !sensors[j].get().get_valid()) continue;
            double expected = particles[i].sensor_readings[j];
            double reading = sensors[j].get().get_reading();
            double sensor_stdev;
            if(reading < 8) sensor_stdev = 0.2;
            else sensor_stdev = reading * 0.05;
            double dev = reading - expected;
            weight *= std::exp(-(dev * dev) / (2 * sensor_stdev * sensor_stdev));
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

static std::uniform_real_distribution<double> sampling_dist;

void resample_particles() {

    std::vector<Particle> newParticles(NUM_PARTICLES);
    sampling_dist = std::uniform_real_distribution<double>(0.0, 1.0 / NUM_PARTICLES);

    double r = sampling_dist(rng);
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

}