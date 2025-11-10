#include "miku/motions.h"
#include "miku/util.h"
#include "config.h"

MovePoint::MovePoint(Point target, double timeout, MovePointParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains drive_gains;
    drive_gains.kP = (params.drive_kP > 0) ? params.drive_kP : drive_gains.kP;
    drive_gains.kI = (params.drive_kI > 0) ? params.drive_kI : drive_gains.kI;
    drive_gains.kD = (params.drive_kD > 0) ? params.drive_kD : drive_gains.kD;
    drive_pid = PID(drive_gains, true, true);

    PIDGains turn_gains;
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
    drive_patience_exit.reset();
}

void MovePoint::update() {
    Point current = Miku.get_position();

    // Calculate angle error in compass frame (0 = +Y)
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    compass_degrees angle_to_point = compass_degrees(miku::atan2(dx, dy));
    if (params.reverse) angle_to_point = (angle_to_point + 180.0).wrap();

    double current_deg = compass_degrees(Miku.get_heading()).wrap();
    double turn_error = (angle_to_point - current_deg).wrap();
    // Distance error
    double drive_error = current.distance_to(target);
    if (params.cutoff > 0 && drive_error < params.cutoff) {
        done = true;
        return;
    }

    standard_radians angle_to_target = miku::atan2(dy, dx);
    standard_radians angle_error = (angle_to_target - Miku.get_pose().theta).wrap();

    drive_error *= std::cos(angle_error);  // Scale error by heading alignment

    drive_patience_exit.update(fabs(turn_error) < 10.0 && fabs(drive_error) < 24.0);

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
    Miku.set_velocities(left_out, right_out);

    if(drive_patience_exit.get_exit() || timer.is_done()) {
        done = true;
        return;
    }
}

bool MovePoint::is_done() {
    return done;
}