#include "miku/motions.h"
#include "miku/util.h"
#include "miku/devices/chassis.h"
#include "system.h"
#include "config.h"

MovePose::MovePose(Point target, compass_degrees heading, float timeout, MovePoseParams params)
    : target(target), target_heading(standard_radians(heading)), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 4.0;
    if(params.k2 < 0) k2 = 6.0;
    if(params.k3 < 0) k3 = 0.5;
}

void MovePose::start() {
    done = false;
    start_time = pros::millis();
    timer.set(timeout);
    timer.reset();
}

void MovePose::update() {
    
    float drive_error = Miku.get_pose().distance_to(target);

    standard_radians robot_heading = Miku.get_heading().wrap();
    if (params.reverse) robot_heading = (robot_heading + M_PI).wrap();

    standard_radians angle_to_target = Miku.get_pose().angle_to(target);
    float gamma = (angle_to_target - robot_heading).wrap();
    float delta = (target_heading - angle_to_target).wrap();

    float v = k1 * drive_error * std::cos(gamma);
    float w;
    if (std::abs(gamma) <= 0.01) {
        w = k2 * gamma + k1 * std::cos(gamma) * (gamma + k3 * delta);
    } else {
        w = k2 * gamma + k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + k3 * delta);
    }

    v = std::clamp(v, -params.max_vel_pct / 100.0f * MAX_VEL, params.max_vel_pct / 100.0f * MAX_VEL);
    w = std::clamp(w, -params.max_vel_pct / 100.0f * MAX_ANG_VEL, params.max_vel_pct / 100.0f * MAX_ANG_VEL);

    if(params.min_vel_pct > 0 && v > 0 && v < fabs(params.min_vel_pct / 100.0f * MAX_VEL)) v = params.min_vel_pct / 100.0f * MAX_VEL;
    float r_vel = v + TRACK_WIDTH * w / 2;
    float l_vel = v - TRACK_WIDTH * w / 2;

    float l_rpm = (l_vel * 60) / (WHEEL_CIRC * GEAR_RATIO);
    float r_rpm = (r_vel * 60) / (WHEEL_CIRC * GEAR_RATIO);

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