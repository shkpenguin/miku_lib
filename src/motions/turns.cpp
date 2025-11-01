#include "motions.h"

TurnHeading::TurnHeading(double target, double timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    Gains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains, true, true);
}

void TurnHeading::start() {
    done = false;
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_small_exit.reset();
    turn_large_exit.reset();
}

void TurnHeading::update() {
    double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
    double error = wrap_angle(target - current_deg, 360);      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    move_motors(output, -output);

    turn_small_exit.update(error);
    turn_large_exit.update(error);

    if(timer.is_done() || turn_small_exit.get_exit() || turn_large_exit.get_exit()) {
        done = true;
        return;
    }
}

bool TurnHeading::is_done() {
    return done;
}

void TurnPoint::start() {
    done = false;
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_small_exit.reset();
    turn_large_exit.reset();
}

void TurnPoint::update() {
    double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
    double target_deg = atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI);
    double error = wrap_angle(target_deg - current_deg, 360);      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    move_motors(output, -output);

    turn_small_exit.update(error);
    turn_large_exit.update(error);

    if(timer.is_done() || turn_small_exit.get_exit() || turn_large_exit.get_exit()) {
        done = true;
        return;
    }
}

bool TurnPoint::is_done() {
    return done;
}