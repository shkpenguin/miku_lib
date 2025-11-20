#include "miku/motions.h"
#include "miku/util.h"
#include "miku/devices/chassis.h"
#include "system.h"
#include "config.h"

MovePose::MovePose(Point target, compass_degrees heading, double timeout, MovePoseParams params)
    : target(target), target_heading(standard_radians(heading)), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 4.0;
    if(params.k2 < 0) k2 = 6.0;
    if(params.k3 < 0) k3 = 0.5;
}

void MovePose::start() {
    done = false;
    timer.set(timeout);
    timer.reset();
}

void MovePose::update() {
    
    double drive_error = Miku.get_pose().distance_to(target);

    standard_radians robot_heading = Miku.get_heading().wrap();
    if (params.reverse) robot_heading = (robot_heading + M_PI).wrap();

    standard_radians angle_to_target = Miku.get_pose().angle_to(target);
    double gamma = (angle_to_target - robot_heading).wrap();
    double delta = (target_heading - angle_to_target).wrap();

    double v = k1 * drive_error * std::cos(gamma);
    double w;
    if (std::abs(gamma) <= 0.01) {
        w = k2 * gamma + k1 * std::cos(gamma) * (gamma + k3 * delta);
    } else {
        w = k2 * gamma + k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + k3 * delta);
    }

    v = std::clamp(v, -params.max_vel, params.max_vel);
    w = std::clamp(w, -params.max_vel, params.max_vel);

    if(!params.reverse && std::abs(v) < params.min_vel && std::abs(v) > 0) v = params.min_vel;
    if(params.reverse && std::abs(v) < params.min_vel && std::abs(v) > 0) v = -params.min_vel;

    double r_vel = v + TRACK_WIDTH * w / 2;
    double l_vel = v - TRACK_WIDTH * w / 2;

    double l_rpm = (l_vel * 60) / (CIRC * GEAR_RATIO);
    double r_rpm = (r_vel * 60) / (CIRC * GEAR_RATIO);

    if (params.reverse) {
        Miku.move_velocity(-r_rpm, -l_rpm);
    } else {
        Miku.move_velocity(l_rpm, r_rpm);
    }

    if(timer.is_done()) {
        done = true;
        return;
    }

    if((params.cutoff > 0 && drive_error < params.cutoff)) {
        done = true;
        Miku.stop();
        return;
    }

    if(drive_error < params.end_cutoff) {
        
        auto settle_motion = new MovePoint(
            Point(target.x, target.y), 
            timer.get_time_left(),
            MovePointParams(
                params.reverse,
                -1.0,
                12000.0,
                1000.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0
            )
        );

        if(!done) queue_after_current(settle_motion);

        done = true;
        Miku.stop();
        return;
    }
}

bool MovePose::is_done() {
    return done;
}   