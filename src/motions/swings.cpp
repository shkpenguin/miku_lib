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
    compass_degrees current_deg = compass_degrees(get_robot_pose().theta);  // convert to degrees
    double error = (target - current_deg).wrap();      // error in degrees

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
    compass_degrees current_deg = compass_degrees(get_robot_pose().theta);  // convert to degrees
    compass_degrees target_deg = compass_degrees(miku::atan2(target.x - get_robot_pose().x, target.y - get_robot_pose().y));
    double error = (target_deg - current_deg).wrap();      // error in degrees

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