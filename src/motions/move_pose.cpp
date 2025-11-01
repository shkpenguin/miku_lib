#include "motions.h"

MovePose::MovePose(Pose target, double timeout, MovePoseParams params)
    : target(target), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 1.0;
    if(params.k2 < 0) k2 = 4.0;
    if(params.k3 < 0) k3 = 1.0;
}

void MovePose::start() {
    done = false;
    timer.set(timeout);
    timer.reset();
    
    target_rad = std::fmod(90 - target.theta, 360) * M_PI / 180;
}

void MovePose::update() {
    
    double drive_error = dist(target.x, target.y, get_pose().x, get_pose().y);

    double robot_heading = std::fmod(get_pose({.standard = true}).theta, 2 * M_PI);
    if (params.reverse) robot_heading = std::fmod(robot_heading + M_PI, 2 * M_PI);

    double gamma = std::remainder(std::atan2(target.y - get_pose().y, target.x - get_pose().x) - robot_heading, 2 * M_PI);
    double delta = std::remainder(std::atan2(target.y - get_pose().y, target.x - get_pose().x) - target_rad, 2 * M_PI);

    double v = k1 * drive_error * std::cos(gamma);
    double w;
    if (std::abs(gamma) <= 0.01) {
        w = k2 * gamma + k1 * std::cos(gamma) * (gamma + k3 * delta);
    } else {
        w = k2 * gamma + k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + k3 * delta);
    }

    if (std::abs(v) > params.max_vel) {
        v *= params.max_vel / std::abs(v);
        w *= params.max_vel / std::abs(v);
    }

    double r_vel = v + TRACK_WIDTH * w / 2;
    double l_vel = v - TRACK_WIDTH * w / 2;

    double max_vel = std::max({std::abs(r_vel), std::abs(l_vel)});
    if (max_vel > 100) {
        r_vel *= 100 / max_vel;
        l_vel *= 100 / max_vel;
    }

    if (params.reverse) {
        double r_volts = drive_lut.get_value(-l_vel);
        double l_volts = drive_lut.get_value(-r_vel);
        move_motors(l_volts, r_volts);
    } else {
        double r_volts = drive_lut.get_value(r_vel);
        double l_volts = drive_lut.get_value(l_vel);
        move_motors(l_volts, r_volts);
    }

    if(timer.is_done()) {
        done = true;
        return;
    }

    if(drive_error < params.cutoff || drive_error < params.end_cutoff) {
        done = true;

        auto settle_motion = std::make_shared<MovePoint>(
            Point(target.x, target.y), 
            timer.get_time_left(),
            MovePointParams(
                params.reverse, 
                false, 
                params.end_cutoff, 
                params.max_vel * 120, 
                params.min_vel * 120
            )
        );

        queue_after_current(settle_motion);

        return;
    }
}

bool MovePose::is_done() {
    return done;
}