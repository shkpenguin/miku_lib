#include "miku/motions.h"
#include "miku/devices/chassis.h"
#include "config.h"
#include "miku/util.h"

TurnHeading::TurnHeading(compass_degrees target, double timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
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
    compass_degrees current_deg = compass_degrees(Miku.get_heading());  // convert to degrees
    compass_degrees error = (target - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    Miku.set_voltages(output, -output);

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
    compass_degrees current_deg = compass_degrees(Miku.get_heading());  // convert to degrees
    compass_degrees target_deg = compass_degrees(atan2(target.x - Miku.get_pose().x, target.y - Miku.get_pose().y) * (180 / M_PI));
    compass_degrees error = (target_deg - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    double output = turn_pid.update(error);

    Miku.set_voltages(output, -output);

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