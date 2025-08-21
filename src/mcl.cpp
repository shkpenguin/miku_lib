#define NUM_PARTICLES 1000
#define INIT_STDEV 2.0
#define SENSOR_STDEV 0.5

#include "api.h"
#include "odom.h"
#include "math.h"
#include <random>
#include <cmath>
#include <vector>
#include <chrono>
#include <fstream>
#include <climits>

class MCLDistance {
    pros::Distance distance_sensor;
    double data = -1;

public:
    Pose offset;
    MCLDistance(int32_t port, Pose offset) : distance_sensor(port), offset(offset) {}
    bool update_reading();
    double get_reading();
};

MCLDistance back(10, Pose(2.5, -6, M_PI)); // in standard angle!!
MCLDistance left(2, Pose(-4.625, 1.375, M_PI/2));
MCLDistance right(9, Pose(4.625, 1.375, -M_PI/2));

bool MCLDistance::update_reading() {

    bool visible = distance_sensor.get_object_size() > 70;

    bool valid = distance_sensor.get_distance() < 100.0;

    if(visible && valid) {
        data = distance_sensor.get_distance() / 25.4;
    }

    return visible && valid;
}

double MCLDistance::get_reading() {
    return data;
}

double get_expected_reading(Pose particle_pose, Pose offset) {

    Pose robot_pose = getPose(true); // get standard pose
    double dx = std::cos(robot_pose.theta + offset.theta);
    double dy = std::sin(robot_pose.theta + offset.theta);

    double dx = std::cos(particle_pose.theta + offset.theta);
    double dy = std::sin(particle_pose.theta + offset.theta);

    float tMin = INT_MAX;

    if (std::abs(dx) > 1e-6f) {
        float t1 = (-72.0 - particle_pose.x)/dx;
        if (t1>0) {
            float y1 = particle_pose.y + t1*dy;
            if (y1>=-72.0 && y1<=72.0) tMin = std::min(tMin, t1);
        }
        float t2 = (72.0 - particle_pose.x)/dx;
        if (t2>0) {
            float y2 = particle_pose.y + t2*dy;
            if (y2>=-72.0 && y2<=72.0) tMin = std::min(tMin, t2);
        }
    }

    if (std::abs(dy) > 1e-6f) {
        float t3 = (-72.0 - particle_pose.y)/dy;
        if (t3>0) {
            float x3 = particle_pose.x + t3*dx;
            if (x3>=-72.0 && x3<=72.0) tMin = std::min(tMin, t3);
        }
        float t4 = (72.0 - particle_pose.y)/dy;
        if (t4>0) {
            float x4 = particle_pose.x + t4*dx;
            if (x4>=-72.0 && x4<=72.0) tMin = std::min(tMin, t4);
        }
    }

    return tMin;

}

std::vector<Particle> particles(NUM_PARTICLES);
std::mt19937_64 rng(std::random_device{}());

void log_mcl() {
    std::ofstream file("log.txt");

    if (!file.is_open()) {
        // Handle error
        return;
    }

    Pose robot_pose = getPose();

    file << NUM_PARTICLES << "," << robot_pose.x << "," << robot_pose.y << "," << robot_pose.theta << ",";
    for(const auto& particle : particles) {
        file << particle.pose.x << "," << particle.pose.y << "," << particle.weight << "\n";
    }
    
}

std::normal_distribution<double> init(0, INIT_STDEV);

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

void initialize_particles() {

    Pose robot_pose = getPose();
    for(int i = 0; i < NUM_PARTICLES; ++i) {
        double rand = init(rng);
        particles[i].pose = Pose(clamp_field(robot_pose.x + rand), clamp_field(robot_pose.y + rand), robot_pose.theta);
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

    log_mcl();
}

#define MAX_ERROR 5.0

void update_particles() {

    double velocity = sqrt(getSpeed().x * getSpeed().x + getSpeed().y * getSpeed().y);
    std::normal_distribution<double> update(0, velocity / 4);

    Pose robot_speed = getSpeed();

    bool useLeft = left.update_reading();
    bool useRight = right.update_reading();
    bool useBack = back.update_reading();

    double total_weight = 0.0; 

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].pose.x = clamp_field(particles[i].pose.x + robot_speed.x + update(rng));
        particles[i].pose.y = clamp_field(particles[i].pose.y + robot_speed.y + update(rng));

        double left_expected = get_expected_reading(particles[i].pose, left.offset);
        double right_expected = get_expected_reading(particles[i].pose, right.offset);
        double back_expected = get_expected_reading(particles[i].pose, back.offset);

        double weight = 1.0;

        double left_dev = fabs(left.get_reading() - left_expected);
        double right_dev = fabs(right.get_reading() - right_expected);
        double back_dev = fabs(back.get_reading() - back_expected);

        if(useLeft && fabs(left_dev) < MAX_ERROR) {
            weight *= std::exp(-(left_dev * left_dev) / (-2 * SENSOR_STDEV * SENSOR_STDEV));
        } 

        if(useRight && fabs(right.get_reading() - right_expected) < MAX_ERROR) {
            weight *= std::exp(-(right_dev * right_dev) / (-2 * SENSOR_STDEV * SENSOR_STDEV));
        }

        if(useBack && fabs(back.get_reading() - back_expected) < MAX_ERROR) {
            weight *= std::exp(-(back_dev * back_dev) / (-2 * SENSOR_STDEV * SENSOR_STDEV));
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

    log_mcl();

}

void resampleParticles() {
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
        newParticles[m] = particles[i];
        newParticles[m].weight = 1.0 / NUM_PARTICLES; // reset weights
    }

    particles = newParticles;

    log_mcl();
}