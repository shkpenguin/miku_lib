#include "miku/motions.hpp"
#include "config.hpp"

MoveDistance::MoveDistance(float target, float timeout, MoveDistanceParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains = drive_gains;
    if(params.drive_kP > 0) gains.kP = params.drive_kP;
    if(params.drive_kI > 0) gains.kI = params.drive_kI;
    if(params.drive_kD > 0) gains.kD = params.drive_kD;
    drive_pid = PID(gains, true, true);
}

void MoveDistance::start() {
    done = false;
    start_time = pros::millis();
    start_left = left_motors.get_average_position();
    start_right = right_motors.get_average_position();
    timer.set(timeout);
    timer.reset();
    drive_pid.reset();
    drive_quick_exit.reset();
    drive_slow_exit.reset();
}

void MoveDistance::update() {
    if(timer.is_done()) {
        done = true;
        Miku.stop();
        return;
    }

    float cur_left = left_motors.get_average_position();
    float cur_right = right_motors.get_average_position();
    float left_delta = cur_left - start_left;
    float right_delta = cur_right - start_right;

    float left_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;
    float right_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;
    float mid_in = (left_in + right_in) / 2.0f;

    float error = (params.reverse ? -1 : 1) * (target - mid_in);

    // cutoff handling
    if(params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        Miku.stop();
        return;
    }

    // finished if we've passed the target or are within half an inch
    if (fabs(error) < 0.5f || (error < 0) != (target < 0)) {
        done = true;
        Miku.stop();
        return;
    }

    if(params.quick_exit) {
        drive_quick_exit.update(fabs(error));
    } else {
        drive_slow_exit.update(fabs(error));
    }

    // use PID on distance error to compute drive voltage
    float drive_out = drive_pid.update(error);
    float limit = params.max_volt_pct / 100.0f * 12000;
    drive_out = std::clamp(drive_out, -limit, limit);
    if(params.min_volt_pct > 0) {
        float minv = params.min_volt_pct / 100.0f * 12000;
        if(drive_out > 0 && drive_out < minv) drive_out = minv;
        if(drive_out < 0 && drive_out > -minv) drive_out = -minv;
    }
    Miku.move_voltage(drive_out, drive_out);

    if((params.quick_exit && drive_quick_exit.get_exit()) ||
       (!params.quick_exit && drive_slow_exit.get_exit())) {
        done = true;
        Miku.stop();
        return;
    }
}

void MoveDistance::stop() {
    done = true;
    Miku.stop();
}

bool MoveDistance::is_done() {
    return done;
}
