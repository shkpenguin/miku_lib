#include "miku/motions.hpp"
#include "miku/util.hpp"
#include "miku/devices/chassis.hpp"
#include "system.hpp"
#include "config.hpp"

MovePose::MovePose(Point target, compass_degrees heading, float timeout, MovePoseParams params)
    : target(target), target_heading(standard_radians(heading)), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 4.0;
    if(params.k2 < 0) k2 = 6.0;
    if(params.k3 < 0) k3 = 0.5;
    if(params.reverse) target_heading = (target_heading + M_PI).wrap();
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

    standard_radians angle_to_target = miku::atan2(target.y - Miku.get_y(), target.x - Miku.get_x()).wrap();
    float gamma = (angle_to_target - robot_heading).wrap();
    float delta = (angle_to_target - target_heading).wrap();

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
        Miku.stop();
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
                params.reverse, // reverse
                -1.0, // cutoff
                100.0, // drive_max_volt_pct
                50.0, // turn_max_volt_pct
                0.0, // min_volt_pct
                1.0, // cos_scale
                -1.0, // drive_kP
                -1.0, // drive_kI
                -1.0, // drive_kD
                -1.0, // turn_kP
                -1.0, // turn_kI
                -1.0 // turn_kD
            )
        );

        if(!done) queue_after_current(settle_motion);

        done = true;
        Miku.stop();
        return;
    }
}

void MovePose::stop() {
    done = true;
    if(params.cutoff < 0) {
        Miku.stop();
    }
}

bool MovePose::is_done() {
    return done;
}   