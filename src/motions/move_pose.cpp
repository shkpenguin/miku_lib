#include "motions.h"

MovePose::MovePose(Point target, compass_degrees heading, double timeout, MovePoseParams params)
    : target(target), target_heading(standard_radians(heading)), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 1.0;
    if(params.k2 < 0) k2 = 4.0;
    if(params.k3 < 0) k3 = 1.0;
}

void MovePose::start() {
    done = false;
    timer.set(timeout);
    timer.reset();
}

void MovePose::update() {
    
    double drive_error = dist(target.x, target.y, get_robot_pose().x, get_robot_pose().y);

    standard_radians robot_heading = get_robot_pose().theta.wrap();
    if (params.reverse) robot_heading = (robot_heading + M_PI).wrap();

    standard_radians angle_to_target = get_robot_pose().angle_to(target);
    double gamma = (angle_to_target - robot_heading).wrap();
    double delta = (target_heading - angle_to_target).wrap();

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
        miku.set_velocities(-r_vel, -l_vel);
    } else {
        miku.set_velocities(l_vel, r_vel);
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