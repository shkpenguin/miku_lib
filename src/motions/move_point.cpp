#include "miku/motions.h"
#include "miku/util.h"
#include "config.h"

MovePoint::MovePoint(Point target, float timeout, MovePointParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains mvt_drive_gains;
    mvt_drive_gains.kP = (params.drive_kP > 0) ? params.drive_kP : drive_gains.kP;
    mvt_drive_gains.kI = (params.drive_kI > 0) ? params.drive_kI : drive_gains.kI;
    mvt_drive_gains.kD = (params.drive_kD > 0) ? params.drive_kD : drive_gains.kD;
    drive_pid = PID(mvt_drive_gains, true, true);

    PIDGains mvt_turn_gains;
    mvt_turn_gains.kP = (params.turn_kP > 0) ? params.turn_kP : turn_gains.kP;
    mvt_turn_gains.kI = (params.turn_kI > 0) ? params.turn_kI : turn_gains.kI;
    mvt_turn_gains.kD = (params.turn_kD > 0) ? params.turn_kD : turn_gains.kD;
    turn_pid = PID(mvt_turn_gains, true, true);
}

void MovePoint::start() {
    done = false;
    start_time = pros::millis();
    drive_pid.reset();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    drive_patience_exit.reset();

    Point current = Miku.get_position();
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    start_side = sign(dx * cos(Miku.get_heading()) + dy * sin(Miku.get_heading()));
}

void MovePoint::update() {
    Point current = Miku.get_position();

    // Calculate angle error in compass frame (0 = +Y)
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    compass_degrees angle_to_point = compass_degrees(miku::atan2(dy, dx));
    if (params.reverse) angle_to_point = (angle_to_point + 180.0f).wrap();

    float current_deg = compass_degrees(Miku.get_heading()).wrap();

    float turn_error = (angle_to_point - current_deg).wrap();
    // Distance error
    float drive_error = current.distance_to(target);
    if (params.cutoff > 0 && drive_error < params.cutoff) {
        done = true;
        return;
    }

    standard_radians angle_error = (miku::atan2(dy, dx) - Miku.get_heading()).wrap();

    drive_patience_exit.update(drive_error);

    // PID outputs
    float drive_out = std::clamp(drive_pid.update(drive_error), -params.drive_max_volt_pct / 100.0f * 12000, params.drive_max_volt_pct / 100.0f * 12000);
    float turn_out = std::clamp(turn_pid.update(turn_error), -params.turn_max_volt_pct / 100.0f * 12000, params.turn_max_volt_pct / 100.0f * 12000);
    drive_out *= std::pow(std::cos(angle_error), params.cos_scale);

    if(fabs(drive_error) < 6.0) {
        int side = sign(dx * cos(Miku.get_heading()) + dy * sin(Miku.get_heading()));
        if(side != start_side) {
            done = true;
            Miku.stop();
            return;
        }
        turn_out *= pow((drive_error / 6.0), 4);
    }

    if(params.min_volt_pct > 0) {
        if(drive_out > 0 && drive_out < fabs(params.min_volt_pct / 100.0f * 12000)) drive_out = params.min_volt_pct / 100.0f * 12000;
        if(drive_out < 0 && drive_out > -fabs(params.min_volt_pct / 100.0f * 12000)) drive_out = -params.min_volt_pct / 100.0f * 12000;
    }

    float left_out = drive_out + turn_out;
    float right_out = drive_out - turn_out;

    // Final motor outputs
    Miku.move_voltage(left_out, right_out);

    if(drive_patience_exit.get_exit() || timer.is_done()) {
        done = true;
        Miku.stop();
        return;
    }
}

bool MovePoint::is_done() {
    return done;
}