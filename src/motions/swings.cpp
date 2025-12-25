#include "miku/motions.hpp"
#include "miku/devices/chassis.hpp"
#include "config.hpp"
#include "miku/util.hpp"

SwingHeading::SwingHeading(compass_degrees target, float timeout, SwingParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void SwingHeading::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_patience_exit.reset();
}

void SwingHeading::update() {
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

    if(params.locked_side == Side::LEFT) {
        right_motors.move_voltage(-output);
    } else {
        left_motors.move_voltage(output);
    }

    turn_patience_exit.update(error);

    if(timer.is_done() || turn_patience_exit.get_exit()) {
        done = true;
        return;
    }
}

void SwingHeading::stop() {
    done = true;
    if(params.cutoff < 0) {
        Miku.stop();
    }
}

bool SwingHeading::is_done() {
    return done;
}

SwingPoint::SwingPoint(Point target, float timeout, SwingParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void SwingPoint::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_patience_exit.reset();
}

void SwingPoint::update() {
    compass_degrees current_deg = compass_degrees(Miku.get_heading());  // convert to degrees
    compass_degrees target_deg = compass_degrees(miku::atan2(target.x - Miku.get_pose().x, target.y - Miku.get_pose().y));
    float error = (target_deg - current_deg).wrap();      // error in degrees

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

    if(params.locked_side == Side::LEFT) {
        right_motors.move_voltage(output);
    } else {
        left_motors.move_voltage(output);
    }

    turn_patience_exit.update(error);

    if(timer.is_done() || turn_patience_exit.get_exit()) {
        done = true;
        return;
    }
}

void SwingPoint::stop() {
    done = true;
    if(params.cutoff < 0) {
        Miku.stop();
    }
}

bool SwingPoint::is_done() {
    return done;
}