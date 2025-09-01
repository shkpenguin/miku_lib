#define NUM_PARTICLES 500
#define INIT_STDEV 2.0
#define SENSOR_STDEV 1.0

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

MCLDistance back(10, Pose(2.5, -6, M_PI));
MCLDistance left(2, Pose(-4.625, 1.375, -M_PI/2));
MCLDistance right(9, Pose(4.625, 1.375, M_PI/2));

bool MCLDistance::update_reading() {

    bool visible = distance_sensor.get_object_size() > 50 || distance_sensor.get_distance() < 100;

    bool valid = distance_sensor.get_distance() < 1800;

    if(visible && valid) {
        data = distance_sensor.get_distance() / 25.4;
    }

    return visible && valid;
}

double MCLDistance::get_reading() {
    return data;
}

double get_expected_reading(Pose particle_pose, Pose offset) {
    double robot_theta = getPose().theta;

    double sensor_x = particle_pose.x + offset.x * cos(robot_theta) - offset.y * sin(robot_theta);
    double sensor_y = particle_pose.y + offset.x * sin(robot_theta) + offset.y * cos(robot_theta);

    // Beam direction: standard math convention, 0 = +x
    double dx = sin(robot_theta + offset.theta);
    double dy = cos(robot_theta + offset.theta);

    double tMin = 1e6;

    // Check X boundaries (-72 to 72)
    if (std::abs(dx) > 1e-6) {
        double t1 = (-72.0 - sensor_x) / dx;
        if (t1 > 0) {
            double y1 = sensor_y + t1 * dy;
            if (y1 >= -72.0 && y1 <= 72.0) tMin = std::min(tMin, t1);
        }
        double t2 = (72.0 - sensor_x) / dx;
        if (t2 > 0) {
            double y2 = sensor_y + t2 * dy;
            if (y2 >= -72.0 && y2 <= 72.0) tMin = std::min(tMin, t2);
        }
    }

    // Check Y boundaries (-72 to 72)
    if (std::abs(dy) > 1e-6) {
        double t3 = (-72.0 - sensor_y) / dy;
        if (t3 > 0) {
            double x3 = sensor_x + t3 * dx;
            if (x3 >= -72.0 && x3 <= 72.0) tMin = std::min(tMin, t3);
        }
        double t4 = (72.0 - sensor_y) / dy;
        if (t4 > 0) {
            double x4 = sensor_x + t4 * dx;
            if (x4 >= -72.0 && x4 <= 72.0) tMin = std::min(tMin, t4);
        }
    }

    double x_intersect = fabs(sensor_x + tMin * dx);
    double y_intersect = fabs(sensor_y + tMin * dy);

    if (y_intersect > 71.0 && y_intersect < 73.0) { // top wall
        if ((x_intersect >= -54.0 && x_intersect <= -42.0) ||
            (x_intersect >= 42.0 && x_intersect <= 54.0)) {
            return -1; // in deadzone
        }
    }

    if (y_intersect > -73.0 && y_intersect < -71.0) { // bottom wall
        if ((x_intersect >= -54.0 && x_intersect <= -42.0) ||
            (x_intersect >= 42.0 && x_intersect <= 54.0)) {
            return -1; // in deadzone
        }
    }

    return tMin;
}

std::vector<Particle> particles(NUM_PARTICLES);
std::mt19937_64 rng(std::random_device{}());

std::ofstream file;
std::ostringstream log_buffer;

void log_mcl() {

    // file << "meow";

    if (!file.is_open()) {
        // Handle error
        return;
    }

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

    // file.open("log.txt");

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
#define MAX_LATERAL_ERROR 6.0

// std::normal_distribution<double> lateral_noise(0, LATERAL_STDEV);
static std::normal_distribution<double> odom_noise;

void update_particles() {

    double velocity = sqrt(getSpeed().x * getSpeed().x + getSpeed().y * getSpeed().y);
    odom_noise = std::normal_distribution<double>(0, velocity / 4);

    Pose robot_speed = getSpeed();
    double robot_theta = getPose(true).theta;
    double sin_theta = sin(robot_theta);
    double cos_theta = cos(robot_theta);

    bool useLeft = left.update_reading();
    bool useRight = right.update_reading();
    bool useBack = back.update_reading();

    double total_weight = 0.0; 
    int invalid_updates = 0;

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].pose.x = clamp_field(
            particles[i].pose.x + 
            robot_speed.x + 
            odom_noise(rng));
        particles[i].pose.y = clamp_field(
            particles[i].pose.y + 
            robot_speed.y + 
            odom_noise(rng));

        double left_expected = get_expected_reading(particles[i].pose, left.offset);
        double right_expected = get_expected_reading(particles[i].pose, right.offset);
        double back_expected = get_expected_reading(particles[i].pose, back.offset);

        bool left_valid = useLeft && left_expected >= 0;
        bool right_valid = useRight && right_expected >= 0;
        bool back_valid = useBack && back_expected >= 0;

        double weight = 1.0;

        double left_dev = fabs(left.get_reading() - left_expected);
        double right_dev = fabs(right.get_reading() - right_expected);
        double back_dev = fabs(back.get_reading() - back_expected);

        left_valid = left_valid && left_dev < MAX_ERROR;
        right_valid = right_valid && right_dev < MAX_ERROR;
        back_valid = back_valid && back_dev < MAX_ERROR;

        int valid_count = 0; // Track how many sensors contributed

        if (left_valid) {
            weight *= std::exp(-(left_dev * left_dev) / (2 * SENSOR_STDEV * SENSOR_STDEV));
            valid_count++;
        }

        if (right_valid) {
            weight *= std::exp(-(right_dev * right_dev) / (2 * SENSOR_STDEV * SENSOR_STDEV));
            valid_count++;
        }

        if (back_valid) {
            weight *= std::exp(-(back_dev * back_dev) / (2 * SENSOR_STDEV * SENSOR_STDEV));
            valid_count++;
        }

        // If no sensors provided valid data, give a neutral but nonzero weight
        if (valid_count == 0) {
            weight = 1e-6;
            invalid_updates++;
        }

        particles[i].weight = weight;
        total_weight += weight;
    }

    if(total_weight == 0.0) {
        double weight = 1.0 / NUM_PARTICLES;
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight = weight;
        }
    } else if(invalid_updates / NUM_PARTICLES > 0.1) {
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight = 1.0 / NUM_PARTICLES;
        }
    } else {
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight /= total_weight;
        }
    }

    // log_mcl();

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

    // Normalize after resampling
    // double total_weight = 0.0;
    // for(const auto &p : newParticles) total_weight += p.weight;
    // for(auto &p : newParticles) p.weight /= total_weight;

    for (auto &p : newParticles) {
        p.weight = 1.0 / NUM_PARTICLES;
    }
    particles = newParticles;
    // log_mcl();
}