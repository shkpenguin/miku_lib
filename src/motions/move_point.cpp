#include "motions.h"

MovePoint::MovePoint(Point target, double timeout, MovePointParams params)
    : target(target), timeout(timeout), params(params) {
    Gains drive_gains;
    drive_gains.kP = (params.drive_kP > 0) ? params.drive_kP : drive_gains.kP;
    drive_gains.kI = (params.drive_kI > 0) ? params.drive_kI : drive_gains.kI;
    drive_gains.kD = (params.drive_kD > 0) ? params.drive_kD : drive_gains.kD;
    drive_pid = PID(drive_gains, true, true);

    Gains turn_gains;
    turn_gains.kP = (params.turn_kP > 0) ? params.turn_kP : turn_gains.kP;
    turn_gains.kI = (params.turn_kI > 0) ? params.turn_kI : turn_gains.kI;
    turn_gains.kD = (params.turn_kD > 0) ? params.turn_kD : turn_gains.kD;
    turn_pid = PID(turn_gains, true, true);
}

void MovePoint::start() {
    done = false;
    drive_pid.reset();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    drive_small_exit.reset();
    drive_large_exit.reset();
}

void MovePoint::update() {
    Point current(get_pose().x, get_pose().y);

    // Calculate angle error in compass frame (0 = +Y)
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double desired_deg = atan2(dx, dy);
    if (params.reverse) desired_deg = wrap_angle(desired_deg + M_PI, 2 * M_PI);

    double current_deg = get_pose().theta;
    double turn_error = wrap_angle(desired_deg - current_deg, 2 * M_PI) * (180.0 / M_PI);  

    // Distance error
    double drive_error = dist(current.x, current.y, target.x, target.y);
    if (params.cutoff > 0 && drive_error < params.cutoff) {
        done = true;
        return;
    }

    double angle_to_target = atan2(dy, dx);
    double angle_error = wrap_angle(angle_to_target - get_pose({.standard = true}).theta, 2 * M_PI);

    drive_error *= std::cos(angle_error);  // Scale error by heading alignment

    drive_small_exit.update(drive_error);
    drive_large_exit.update(drive_error);

    // PID outputs
    double drive_out = std::clamp(drive_pid.update(drive_error), -params.max_speed, params.max_speed);
    double turn_out = std::clamp(turn_pid.update(turn_error), -params.max_speed, params.max_speed);
    if(fabs(drive_error) < 6.0) {
        turn_out *= pow((drive_error / 6.0), 3);
    }

    if(!params.reverse && drive_out < fabs(params.min_speed) && drive_out > 0) drive_out = params.min_speed;
    if(params.reverse && drive_out < fabs(params.min_speed) && drive_out > 0) drive_out = -params.min_speed;

    double left_out = drive_out + turn_out;
    double right_out = drive_out - turn_out;

    // Final motor outputs 
    move_motors(left_out, right_out);

    if(drive_small_exit.get_exit() || drive_large_exit.get_exit() || timer.is_done()) {
        done = true;
        return;
    }
}

bool MovePoint::is_done() {
    return done;
}