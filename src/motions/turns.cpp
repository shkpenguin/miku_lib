#include "miku/motions.h"
#include "miku/devices/chassis.h"
#include "config.h"
#include "miku/util.h"

TurnHeading::TurnHeading(compass_degrees target, float timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void TurnHeading::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_patience_exit.reset();
}

void TurnHeading::update() {
    compass_degrees current_deg = compass_degrees(Miku.get_heading());  // convert to degrees
    float error = (target - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output = turn_pid.update(error);
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    Miku.move_voltage(output, -output);

    if(timer.is_done() || turn_patience_exit.get_exit()) {
        done = true;
        Miku.stop();
        return;
    }
}

bool TurnHeading::is_done() {
    return done;
}

TurnPoint::TurnPoint(Point target, float timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains, true, true);
}

void TurnPoint::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_patience_exit.reset();
}

void TurnPoint::update() {
    compass_degrees current_deg = compass_degrees(Miku.get_heading());  // convert to degrees
    compass_degrees target_deg = compass_degrees(atan2(target.y - Miku.get_pose().y, target.x - Miku.get_pose().x) * (180 / M_PI));
    compass_degrees error = (target_deg - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output = turn_pid.update(error);
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    Miku.move_voltage(output, -output);

    turn_patience_exit.update(error);
    if(timer.is_done() || turn_patience_exit.get_exit()) {
        done = true;
        Miku.stop();
        return;
    }
}

bool TurnPoint::is_done() {
    return done;
}