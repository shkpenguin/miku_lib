#include "motions.h"

void SwingHeading::start() {
    done = false;
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_small_exit.reset();
    turn_large_exit.reset();
}

void SwingHeading::update() {
    double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
    double error = wrap_angle(target - current_deg, 360);      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    if(params.locked_side == Side::LEFT) {
        right_motors.move_voltage(output);
    } else {
        left_motors.move_voltage(output);
    }

    turn_small_exit.update(error);
    turn_large_exit.update(error);

    if(timer.is_done() || turn_small_exit.get_exit() || turn_large_exit.get_exit()) {
        done = true;
        return;
    }
}

bool SwingHeading::is_done() {
    return done;
}

void SwingPoint::start() {
    done = false;
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_small_exit.reset();
    turn_large_exit.reset();
}

void SwingPoint::update() {
    double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
    double target_deg = atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI);
    double error = wrap_angle(target_deg - current_deg, 360);      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    if(params.locked_side == Side::LEFT) {
        right_motors.move_voltage(output);
    } else {
        left_motors.move_voltage(output);
    }

    turn_small_exit.update(error);
    turn_large_exit.update(error);

    if(timer.is_done() || turn_small_exit.get_exit() || turn_large_exit.get_exit()) {
        done = true;
        return;
    }
}

bool SwingPoint::is_done() {
    return done;
}