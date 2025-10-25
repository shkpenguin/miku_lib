#include "timer.h"
#include "config.h"
#include "motions.h"
#include "geometry.h"
#include "pid.h"
#include "lut.h"

pros::Mutex motion_mutex;

bool motion_running = false;
bool motion_queued = false;
double distance_traveled = -1;

void move_motors(double l, double r) {
    left_motors.move_voltage(l);
    right_motors.move_voltage(r);
}

void stop_motors() {
    left_motors.move_voltage(0);
    right_motors.move_voltage(0);
    pros::delay(10);
}

void set_drive_brake(pros::motor_brake_mode_e mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

void wait_until_done() {
    do {
        pros::delay(10);
    } while(distance_traveled != -1);
}

void wait_until_within(Point target, double threshold) {
    do {
        pros::delay(10);
    } while (std::hypot(get_pose().x - target.x, get_pose().y - target.y) > threshold && distance_traveled != -1);
}

void wait_until_within(double target_angle, double threshold) {
    do {
        pros::delay(10);
    } while (fabs(wrap_angle(get_pose({.degrees = true}).theta - target_angle, 360)) > threshold && distance_traveled != -1);
}

void request_motion_start() {
    if(motion_running) motion_queued = true;
    else {
        motion_running = true;
    }

    motion_mutex.take(TIMEOUT_MAX);
}

void end_motion() {
    motion_running = motion_queued;
    motion_queued = false;
    motion_mutex.give();
}

void cancel_motion() {
    motion_running = false; 
    pros::delay(10);
}

void stop_motion() {
    end_motion();
    distance_traveled = -1;
    stop_motors();
}

void cancel_all_motions() {
    motion_running = false;
    motion_queued = false;
    pros::delay(10);
}

bool get_motion_running() {
    return distance_traveled != -1;
};

void turn_heading(double target, double timeout, TurnParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task([target, timeout, params]() mutable { 
            params.async = false;
            turn_heading(target, timeout, params); 
        });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;

    target = wrap_angle(target, 360);
    if (params.reverse) target = wrap_angle(target + 180, 360);

    double initial_error = fabs(wrap_angle(target - get_pose({.degrees = true}).theta, 360));

    // PID tuned in degrees
    Gains turn_gains = Gains(
        turn_kp_lut.get_value(initial_error), 
        0.0, 
        turn_kd_lut.get_value(initial_error)
    );

    PID turn_pid(turn_gains, 0.0, true);

    turn_small_exit.reset();
    turn_large_exit.reset();

    uint32_t prev_time = pros::millis();

    while (!timer.isDone() && !turn_small_exit.getExit() && !turn_large_exit.getExit() && motion_running) {
        double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
        double error = wrap_angle(target - current_deg, 360);      // error in degrees

        if (params.cutoff > 0 && fabs(error) < params.cutoff) break;

        double output = turn_pid.update(error);

        move_motors(output, -output);

        turn_small_exit.update(error);
        turn_large_exit.update(error);

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void turn_point(Point target, double timeout, TurnParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task([target, timeout, params]() mutable { 
            params.async = false;
            turn_point(target, timeout, params); 
        });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;

    double target_deg;

    turn_small_exit.reset();
    turn_large_exit.reset();

    uint32_t prev_time = pros::millis();

    double initial_error = fabs(atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI) - get_pose({.degrees = true}).theta);

    // PID tuned in degrees
    Gains turn_gains = Gains(
        turn_kp_lut.get_value(initial_error), 
        0.0,
        turn_kd_lut.get_value(initial_error)
    );

    PID turn_pid(turn_gains, 0.0, true);

    while (!timer.isDone() && !turn_small_exit.getExit() && !turn_large_exit.getExit() && motion_running) {
        double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
        double target_deg = atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI);
        double error = wrap_angle(target_deg - current_deg, 360);      // error in degrees

        if (params.cutoff > 0 && fabs(error) < params.cutoff) break;

        double output = turn_pid.update(error);
        std::clamp(output, -params.max_speed, params.max_speed);
        if (fabs(output) < params.min_speed) output = (output < 0 ? -params.min_speed : params.min_speed);

        move_motors(output, -output);

        turn_small_exit.update(error);
        turn_large_exit.update(error);

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void swing_heading(double target, double timeout, Side locked_side, SwingParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task([target, timeout, locked_side, params]() mutable { 
            params.async = false;
            swing_heading(target, timeout, locked_side, params); 
        });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;

    target = wrap_angle(target, 360);
    if (params.reverse) target = wrap_angle(target + 180, 360);

    double initial_error = fabs(wrap_angle(target - get_pose({.degrees = true}).theta, 360));
    if(locked_side == Side::AUTO) {
        // if turning left, lock right side, else lock left side
        if(initial_error < 0) {
            locked_side = Side::RIGHT;
        } else {
            locked_side = Side::LEFT;
        }
    }

    if(params.hold) {
        if(locked_side == Side::LEFT) {
            left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        } else {
            right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        }
    }

    // PID tuned in degrees
    Gains turn_gains = Gains(
        turn_kp_lut.get_value(initial_error), 
        0.0, 
        turn_kd_lut.get_value(initial_error)
    );

    PID turn_pid(turn_gains, 0.0, true);

    turn_small_exit.reset();
    turn_large_exit.reset();

    uint32_t prev_time = pros::millis();

    while (!timer.isDone() && !turn_small_exit.getExit() && !turn_large_exit.getExit() && motion_running) {
        double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
        double error = wrap_angle(target - current_deg, 360);      // error in degrees

        if (params.cutoff > 0 && fabs(error) < params.cutoff) break;

        double output = turn_pid.update(error);

        if(locked_side == Side::LEFT) {
            right_motors.move_voltage(output);
        } else {
            left_motors.move_voltage(output);
        }

        turn_small_exit.update(error);
        turn_large_exit.update(error);

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    set_drive_brake(DEFAULT_AUTONOMOUS_BRAKE_MODE);

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void swing_point(Point target, double timeout, Side locked_side, SwingParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task([target, timeout, locked_side, params]() mutable {
            params.async = false;
            swing_point(target, timeout, locked_side, params);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;

    if(locked_side == Side::AUTO) {
        double angle_to_target = atan2(target.y - get_pose().y, target.x - get_pose().x);
        double angle_error = wrap_angle(angle_to_target - get_pose({.standard = true}).theta, 2 * M_PI);
        if(angle_error < 0) {
            locked_side = Side::RIGHT;
        } else {
            locked_side = Side::LEFT;
        }
    }

    if(locked_side == Side::LEFT) {
        left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    } else {
        right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }

    double target_deg;

    turn_small_exit.reset();
    turn_large_exit.reset();

    uint32_t prev_time = pros::millis();

    double initial_error = fabs(atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI) - get_pose({.degrees = true}).theta);

    // PID tuned in degrees
    Gains turn_gains = Gains(
        turn_kp_lut.get_value(initial_error), 
        0.0,
        turn_kd_lut.get_value(initial_error)
    );

    PID turn_pid(turn_gains, 0.0, true);

    while (!timer.isDone() && !turn_small_exit.getExit() && !turn_large_exit.getExit() && motion_running) {
        double current_deg = get_pose({.degrees = true}).theta;  // convert to degrees
        double target_deg = atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI);
        double error = wrap_angle(target_deg - current_deg, 360);      // error in degrees

        if (params.cutoff > 0 && fabs(error) < params.cutoff) break;

        double output = turn_pid.update(error);
        std::clamp(output, -params.max_speed, params.max_speed);
        if (fabs(output) < params.min_speed) output = (output < 0 ? -params.min_speed : params.min_speed);

        if(locked_side == Side::LEFT) {
            right_motors.move_voltage(-output);
        } else {
            left_motors.move_voltage(output);
        }

        turn_small_exit.update(error);
        turn_large_exit.update(error);

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    set_drive_brake(DEFAULT_AUTONOMOUS_BRAKE_MODE);

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void move_point(Point target, double timeout, MovePointParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task move_task([target, timeout, params]() mutable {
            params.async = false;
            move_point(target, timeout, params);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    distance_traveled = 0;
    Timer timer(timeout);

    Gains mtp_gains = params.angular_speed != -1.0 ? 
        Gains(params.angular_speed, 0.0, 0) : drive_gains;
    PID drive_pid(drive_gains);
    
    double initial_error = fabs(atan2(target.x - get_pose().x, target.y - get_pose().y) * (180 / M_PI) - get_pose({.degrees = true}).theta);

    // PID tuned in degrees
    Gains turn_gains = Gains(
        turn_kp_lut.get_value(initial_error), 
        0.0, 
        turn_kd_lut.get_value(initial_error)
    );

    PID turn_pid(turn_gains, 0.0, true);

    drive_small_exit.reset();
    drive_large_exit.reset();

    double prev_drive_out = 0;
    double prev_turn_out = 0;

    uint32_t prev_time = pros::millis();

    while (!drive_small_exit.getExit() && !drive_large_exit.getExit() && !timer.isDone()) {
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
        if (params.cutoff > 0 && drive_error < params.cutoff) break;

        double angle_to_target = atan2(dy, dx);
        double angle_error = wrap_angle(angle_to_target - get_pose({.standard = true}).theta, 2 * M_PI);

        drive_error *= std::cos(angle_error);  // Scale error by heading alignment

        drive_small_exit.update(drive_error);
        drive_large_exit.update(drive_error);

        // PID outputs
        double drive_out = std::clamp(drive_pid.update(drive_error), -params.max_speed, params.max_speed);
        double turn_out = std::clamp(turn_pid.update(turn_error), -params.max_speed, params.max_speed);
        if(fabs(drive_error) < 2.0) turn_out = 0;
        else if(fabs(drive_error) < 6.0) {
            turn_out *= (drive_error / 6.0);
        }

        if(!params.reverse && drive_out < fabs(params.min_speed) && drive_out > 0) drive_out = params.min_speed;
        if(params.reverse && drive_out < fabs(params.min_speed) && drive_out > 0) drive_out = -params.min_speed;

        prev_drive_out = drive_out;
        prev_turn_out = turn_out;

        double left_out = drive_out + turn_out;
        double right_out = drive_out - turn_out;

        // Final motor outputs 
        move_motors(left_out, right_out);

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void move_pose(Pose target, double timeout, MovePoseParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task move_task([target, timeout, params]() mutable {
            params.async = false;
            move_pose(target, timeout, params);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    distance_traveled = 0;

    double theta = std::fmod(90 - target.theta, 360) * M_PI / 180;
    Timer timer(timeout);

    drive_small_exit.reset();
    drive_large_exit.reset();

    double k1 = params.distance_weight;
    double k2 = 4.0;
    double k3 = params.angular_speed;

    uint32_t prev_time = pros::millis();

    while (!drive_small_exit.getExit() && !drive_large_exit.getExit() && !timer.isDone()) {

        double rho = dist(target.x, target.y, get_pose().x, get_pose().y);
        if (rho < params.cutoff || rho < params.end_cutoff) break;

        double rh = std::fmod(get_pose({.standard = true}).theta, 2 * M_PI);
        if (params.reverse) rh = std::fmod(rh + M_PI, 2 * M_PI);
        
        double gamma = std::remainder(std::atan2(target.y - get_pose().y, target.x - get_pose().x) - rh, 2 * M_PI);
        double delta = std::remainder(std::atan2(target.y - get_pose().y, target.x - get_pose().x) - theta, 2 * M_PI);

        double v = k1 * rho * std::cos(gamma);
        double w;
        if (std::abs(gamma) <= 0.01) {
            w = k2 * gamma + k1 * std::cos(gamma) * (gamma + k3 * delta);
        } else {
            w = k2 * gamma + k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + k3 * delta);
        }

        if (std::abs(v) > params.max_vel) {
            v *= params.max_vel / std::abs(v);
            w *= params.max_vel / std::abs(v);
        }

        double r_vel = v + TRACK_WIDTH * w / 2;
        double l_vel = v - TRACK_WIDTH * w / 2;

        double max_vel = std::max({std::abs(r_vel), std::abs(l_vel)});
        if (max_vel > 100) {
            r_vel *= 100 / max_vel;
            l_vel *= 100 / max_vel;
        }

        if (params.reverse) {
            double r_volts = drive_lut.get_value(-l_vel);
            double l_volts = drive_lut.get_value(-r_vel);
            move_motors(l_volts, r_volts);
        } else {
            double r_volts = drive_lut.get_value(r_vel);
            double l_volts = drive_lut.get_value(l_vel);
            move_motors(l_volts, r_volts);
        }
        
        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    if(params.cutoff > 0 && dist(target.x, target.y, get_pose().x, get_pose().y) < params.cutoff) {
        distance_traveled = -1;
        stop_motors();
        end_motion();
        return;
    }

    stop_motors();

    end_motion();

    distance_traveled = 0;

    move_point(Point(target.x, target.y), timer.getTimeLeft(), MovePointParams(params.reverse, params.async, params.cutoff, params.max_vel * 120, params.min_vel * 120));

}

void move_time(double volts, double timeout, bool async) {
    request_motion_start();
    if (!motion_running) return;

    if (async) {
        pros::Task move_task([volts, timeout]() mutable {
            move_time(volts, timeout, false);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    distance_traveled = 0;
    Timer timer(timeout);

    uint32_t prev_time = pros::millis();

    while (!timer.isDone() && motion_running) {
        move_motors(volts, volts);
        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    stop_motors();

    distance_traveled = -1;
    end_motion();
}

void ramsete(std::vector<Waypoint> waypoints, double timeout, RamseteParams params) {

    request_motion_start();
    if (!motion_running) return;

    if (params.async) {
        pros::Task([waypoints, timeout, params]() mutable { 
            params.async = false;
            ramsete(waypoints, timeout, params);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    double zeta = 0.7;
    double time_multi = 1.2;
    double b = params.angular_speed;

    bool isFinished = false;

    Timer timer(timeout);

    int waypoint_num = waypoints.size();
    double end_time = waypoints.back().t / time_multi;

    double end_x = waypoints.back().x;
    double end_y = waypoints.back().y;

    int current_waypoint = 0;
    int closest_waypoint = 0;
    double time_passed = 0;
    double time_ahead = 0;

    distance_traveled = 0; // Track "progress" like other motions

    drive_large_exit.reset();
    drive_small_exit.reset();

    uint32_t prev_time = pros::millis();

    while (!drive_small_exit.getExit() && !drive_large_exit.getExit() && !timer.isDone() &&
          time_passed < end_time - time_ahead ||
          (time_passed < end_time - time_ahead - 1000 && !timer.isDone())) {

        double robot_x = get_pose().x;
        double robot_y = get_pose().y;

        double distance_to_end = dist(end_x, end_y, get_pose().x, get_pose().y);
        if (distance_to_end < params.cutoff || distance_to_end < params.end_cutoff) break;

        double robot_h = get_pose({.standard = true}).theta;
        if (params.reverse) robot_h = fmod(robot_h + M_PI, 2 * M_PI);

        // Update closest waypoint
        double cutoff_dist = dist(robot_x, robot_y, waypoints[closest_waypoint].x, waypoints[closest_waypoint].y);
        while (closest_waypoint < waypoint_num - 2) {
            double search_dist = dist(robot_x, robot_y, waypoints[closest_waypoint + 1].x,
                                      waypoints[closest_waypoint + 1].y);
            if (search_dist >= cutoff_dist) break;
            closest_waypoint++;
            cutoff_dist = search_dist;
        }

        // Update current waypoint by time
        time_passed = timer.getTimePassed();
        while (current_waypoint < waypoint_num - 1 &&
               time_passed + time_ahead > waypoints[current_waypoint].t / time_multi) {
            current_waypoint++;
        }
        if (closest_waypoint > current_waypoint) current_waypoint = closest_waypoint;

        // Extract target waypoint
        const auto &wp = waypoints[current_waypoint];
        double target_x = wp.x, target_y = wp.y;
        double target_h = wp.theta;
        double target_v = wp.linvel, target_w = wp.angvel;

        double dx = target_x - robot_x;
        double dy = target_y - robot_y;

        double error_x =  std::cos(robot_h) * dx + std::sin(robot_h) * dy;
        double error_y = -std::sin(robot_h) * dx + std::cos(robot_h) * dy;
        double error_h = angle_error(target_h, robot_h);

        double k = 2 * zeta * std::sqrt(target_w * target_w + b * target_v * target_v);
        double u1 = -k * error_x;
        double u2 = (std::abs(error_h) <= 0.01) ? (-b * target_v * error_y - k * error_h)
                                                : (-b * target_v * std::sin(error_h) / error_h * error_y - k * error_h);
        double v = target_v * std::cos(error_h) - u1;
        double w = target_w - u2;

        double percent_v = v / (0.01 * MAX_RPM / 60 * GEAR_RATIO * CIRC);
        double percent_w = w * TRACK_WIDTH / (0.01 * MAX_RPM / 60 * GEAR_RATIO * CIRC);

        double r_vel = percent_v + percent_w / 2;
        double l_vel = percent_v - percent_w / 2;

        // Limit motor outputs
        double max_vel = std::max(std::abs(r_vel), std::abs(l_vel));
        if (max_vel > 100) {
            r_vel *= 100 / max_vel;
            l_vel *= 100 / max_vel;
        }

        double r_volts, l_volts;

        if(params.reverse) {
            r_volts = drive_lut.get_value(-l_vel);
            l_volts = drive_lut.get_value(-r_vel);
        } else {
            r_volts = drive_lut.get_value(r_vel);
            l_volts = drive_lut.get_value(l_vel);
        }

        move_motors(l_volts, r_volts);

        distance_traveled += std::abs(error_x) + std::abs(error_y); // crude progress metric

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

    if(params.cutoff > 0 && dist(end_x, end_y, get_pose().x, get_pose().y) < params.cutoff) {
        distance_traveled = -1;
        end_motion();
        return;
    }

    end_motion();

    distance_traveled = 0;

    move_point(Point(end_x, end_y), timer.getTimeLeft(), MovePointParams(params.reverse, params.async, params.cutoff, params.max_vel * 120, params.min_vel * 120));

}