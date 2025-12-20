#include "api.h"
#include "miku/miku-api.h"

std::mt19937_64 rng(std::random_device{}());

ParticleFilter::ParticleFilter(std::vector<std::shared_ptr<miku::Distance>> sensors)
    : distance_sensors(sensors) {
    for(auto& particle : particles) {
        particle.sensor_readings.resize(distance_sensors.size(), WallEstimate{NOT_IN_FIELD, 0.0f});
    }
}


void ParticleFilter::set_max_distance_error(float error) {
    max_error = error;
}
void ParticleFilter::set_min_odom_noise(float noise) {
    min_odom_noise = noise;
}

// Given a particle position and a distance sensor, compute the expected wall intersection
// using the project's sensor offset convention: offset_y is forward (+Y), offset_x is right (+X).
// Sensor world position (with robot heading theta) is:
//   sensor_x = robot_x + offset_x * sin(theta) + offset_y * cos(theta)
//   sensor_y = robot_y - offset_x * cos(theta) + offset_y * sin(theta)
// The ray direction (dx,dy) is chosen based on sensor orientation (front/back/left/right).
// Return value is the WallEstimate (wall id and distance along the ray t).
auto get_expected_reading = [](Point particle_position, std::shared_ptr<miku::Distance> sensor, float cos_theta, float sin_theta) {

    float sensor_y = particle_position.y - sensor->offset_x * cos_theta + sensor->offset_y * sin_theta;
    float sensor_x = particle_position.x + sensor->offset_x * sin_theta + sensor->offset_y * cos_theta;
    
    float dx, dy;

    switch (sensor->orientation) {
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

    WallID intersect;
    float tMin = std::numeric_limits<float>::infinity();

    // Check vertical walls
    if (fabs(dx) > 1e-6) {
        float t1 = (-HALF_FIELD - sensor_x) / dx;
        if (t1 > 0) {
            float y1 = sensor_y + t1 * dy;
            if (t1 > 0 && y1 >= -HALF_FIELD && y1 <= HALF_FIELD && t1 < tMin) {
                tMin = t1;
                intersect = (dx < 0) ? LEFT_WALL : RIGHT_WALL;
            }
        }
        float t2 = (HALF_FIELD - sensor_x) / dx;
        if (t2 > 0) {
            float y2 = sensor_y + t2 * dy;
            if (t2 > 0 && y2 >= -HALF_FIELD && y2 <= HALF_FIELD && t2 < tMin) {
                tMin = t2;
                intersect = (dx < 0) ? LEFT_WALL : RIGHT_WALL;
            }
        }
    }

    // Check horizontal walls
    if (fabs(dy) > 1e-6) {
        float t3 = (-HALF_FIELD - sensor_y) / dy;
        if (t3 > 0) {
            float x3 = sensor_x + t3 * dx;
            if (t3 > 0 && x3 >= -HALF_FIELD && x3 <= HALF_FIELD && t3 < tMin) {
                tMin = t3;
                intersect = (dy < 0) ? BOTTOM_WALL : TOP_WALL;
            }
        }
        float t4 = (HALF_FIELD - sensor_y) / dy;
        if (t4 > 0) {
            float x4 = sensor_x + t4 * dx;
            if (t4 > 0 && x4 >= -HALF_FIELD && x4 <= HALF_FIELD && t4 < tMin) {
                tMin = t4;
                intersect = (dy < 0) ? BOTTOM_WALL : TOP_WALL;
            }
        }
    }

    if (!std::isfinite(tMin)) return WallEstimate{NOT_IN_FIELD, 0}; // no valid hit

    float x_intersect = sensor_x + tMin * dx;
    float y_intersect = sensor_y + tMin * dy;

    // Deadzone check (example for top/bottom walls)
    if (fabs(y_intersect - HALF_FIELD) < 1.0 || fabs(y_intersect + HALF_FIELD) < 1.0) {
        if ((x_intersect >= -54.0 && x_intersect <= -42.0) ||
            (x_intersect >=  42.0 && x_intersect <=  54.0)) {
            return WallEstimate{BAD_INTERSECT, 0}; // in deadzone
        }
    }

    return WallEstimate{intersect, tMin}; // distance along ray

};

void miku::Chassis::distance_reset(Point estimate, float particle_spread) {

    std::vector<WallEstimate> expected_readings;
    float cos_theta = cos(Miku.get_heading());
    float sin_theta = sin(Miku.get_heading());

    for(const auto& sensor : pf->distance_sensors) {
        expected_readings.push_back(
            get_expected_reading(
                Point(estimate.x, estimate.y),
                sensor,
                cos_theta,
                sin_theta));
    }

    float sum_weight_x = 0.0f;
    float sum_weight_y = 0.0f;
    float acc_x = 0.0f;
    float acc_y = 0.0f;

    for(size_t i = 0; i < pf->distance_sensors.size(); ++i) {

        std::shared_ptr<miku::Distance> sensor = pf->distance_sensors[i];

        if(!sensor->get_enabled()) continue;
        if(!sensor->get_valid()) continue;

        WallEstimate expected = expected_readings[i];
        if(expected.wall_id == BAD_INTERSECT || expected.wall_id == NOT_IN_FIELD) continue;

        float reading = sensor->get_reading();
        if(fabs(reading - expected.distance) > pf->max_error) continue;

        float corrected_x = estimate.x;
        float corrected_y = estimate.y;

        // Compute sensor offset in world frame using the same convention as get_expected_reading:
        //   o_x = offset_x * sin(theta) + offset_y * cos(theta)
        //   o_y = -offset_x * cos(theta) + offset_y * sin(theta)
        float o_x = sensor->offset_x * sin_theta + sensor->offset_y * cos_theta;
        float o_y = -sensor->offset_x * cos_theta + sensor->offset_y * sin_theta;

        float dx, dy;
        switch (sensor->orientation) {
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

        switch(expected.wall_id) {
            case LEFT_WALL:
                // sensor_x + reading*dx = -HALF_FIELD  => robot_x = -HALF_FIELD - reading*dx - o_x
                corrected_x = -HALF_FIELD - reading * dx - o_x;
                break;

            case RIGHT_WALL:
                // sensor_x + reading*dx = HALF_FIELD  => robot_x = HALF_FIELD - reading*dx - o_x
                corrected_x = HALF_FIELD - reading * dx - o_x;
                break;

            case TOP_WALL:
                // sensor_y + reading*dy = HALF_FIELD  => robot_y = HALF_FIELD - reading*dy - o_y
                corrected_y = HALF_FIELD - reading * dy - o_y;
                break;

            case BOTTOM_WALL:
                // sensor_y + reading*dy = -HALF_FIELD => robot_y = -HALF_FIELD - reading*dy - o_y
                corrected_y = -HALF_FIELD - reading * dy - o_y;
                break;

            default:
                break;
        }

        // accumulate equally-weighted estimates (no confidence bias)
        if(expected.wall_id == LEFT_WALL || expected.wall_id == RIGHT_WALL) {
            acc_x += corrected_x;
            sum_weight_x += 1.0f;
        }

        if(expected.wall_id == TOP_WALL || expected.wall_id == BOTTOM_WALL) {
            acc_y += corrected_y;
            sum_weight_y += 1.0f;
        }

    }

    Point corrected_pose = estimate;

    if(sum_weight_x > 0.0f) {
        corrected_pose.x = acc_x / sum_weight_x;
    }

    if(sum_weight_y > 0.0f) {
        corrected_pose.y = acc_y / sum_weight_y;
    }

    set_position(Point(
        clamp_field(corrected_pose.x),
        clamp_field(corrected_pose.y)
    ));

    if(particle_spread > 0.0) {
        pf->set_particles_uniform(Point(corrected_pose.x, corrected_pose.y), particle_spread);
    } else {
        pf->set_particles_point(Point(corrected_pose.x, corrected_pose.y));
    }

}

// compute weighted average position of particles
Point ParticleFilter::get_current_belief() {
    float x = 0.0;
    float y = 0.0;
    float total_weight = 0.0;

    for(const auto& particle : particles) {
        x += particle.position.x * particle.weight;
        y += particle.position.y * particle.weight;
        total_weight += particle.weight;
    }

    if(total_weight > 0) {
        x /= total_weight;
        y /= total_weight;
    }

    return Point(x, y);
}

void ParticleFilter::set_particles_point(Point center) {

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = center;
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

void ParticleFilter::set_particles_uniform(Point center, float length) {

    std::uniform_real_distribution<float> dist(length / -2, length / 2);

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = Point(
            clamp_field(center.x + dist(rng)), 
            clamp_field(center.y + dist(rng))
        );
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

static std::normal_distribution<float> odom_noise;

// todo: increase speed with simd
void ParticleFilter::update_previous_belief(Pose robot_speed) {

    Point prev_belief = Miku.get_position();
    Miku.set_position(Point(
        clamp_field(prev_belief.x + robot_speed.x),
        clamp_field(prev_belief.y + robot_speed.y)
    ));

    float velocity = robot_speed.magnitude();
    float odom_stdev = std::max(velocity / 4, min_odom_noise);
    odom_noise = std::normal_distribution<float>(0, odom_stdev);

    for(auto &particle : particles) {
        particle.position.x = clamp_field(particle.position.x + robot_speed.x + odom_noise(rng));
        particle.position.y = clamp_field(particle.position.y + robot_speed.y + odom_noise(rng));
    }

}

void ParticleFilter::update_particle_weights() {

    standard_radians robot_theta = Miku.get_heading();
    float sin_theta = sin(robot_theta);
    float cos_theta = cos(robot_theta);

    std::vector<bool> valid_sensors(distance_sensors.size(), true);

    // max error check
    for(size_t i = 0; i < distance_sensors.size(); ++i) {
        float expected = get_expected_reading(
            Miku.get_position(),
            distance_sensors[i],
            cos_theta, 
            sin_theta).distance;
        if(fabs(expected - distance_sensors[i]->get_reading()) > max_error) valid_sensors[i] = false;
    }

    // check for particles with invalid readings
    for(size_t i = 0; i < particles.size(); ++i) {
        for(size_t j = 0; j < distance_sensors.size(); ++j) {
            particles[i].sensor_readings[j] = get_expected_reading(
                particles[i].position, 
                distance_sensors[j],
                cos_theta,
                sin_theta);
            if(particles[i].sensor_readings[j].wall_id == BAD_INTERSECT) valid_sensors[j] = false;
        }
    }

    for(auto &sensor : distance_sensors) {
        sensor->update_reading();
    }

    float total_weight = 0.0;

    for(int i = 0; i < NUM_PARTICLES; ++i) {

        float weight = 1.0;

        for(int j = 0; j < distance_sensors.size(); ++j) {
            if(!valid_sensors[j] || !distance_sensors[j]->get_enabled() || !distance_sensors[j]->get_valid()) continue;
            WallEstimate expected = particles[i].sensor_readings[j];
            if(expected.wall_id == NOT_IN_FIELD) {
                weight *= 0.0;
                continue;
            }
            float reading = distance_sensors[j]->get_reading();
            float dev = reading - expected.distance;
            if(fabs(dev) > max_error) {
                weight *= 0.0;
                continue;
            }
            float sensor_stdev;
            if(reading < 8) sensor_stdev = 0.1;
            else sensor_stdev = reading * 0.02;
            weight *= std::exp(-(dev * dev) / (2 * sensor_stdev * sensor_stdev));
        }

        particles[i].weight = weight;
        total_weight += weight;
    }

    if(total_weight == 0.0) {
        float weight = 1.0 / NUM_PARTICLES;
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight = weight;
        }
    } else {
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight /= total_weight;
        }
    }

}

static std::uniform_real_distribution<float> sampling_dist;

void ParticleFilter::resample_particles() {

    std::vector<Particle> newParticles(NUM_PARTICLES);
    sampling_dist = std::uniform_real_distribution<float>(0.0, 1.0 / NUM_PARTICLES);

    float r = sampling_dist(rng);
    float c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; m++) {
        float U = r + (float)m / NUM_PARTICLES;
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