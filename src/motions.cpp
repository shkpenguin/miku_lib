#include "motions.h"
#include "util.h"
#include "odom.h"
#include "config.h"
#include "timer.h"

pros::Mutex motion_mutex;

bool motion_running = false;
bool motion_queued = false;
double distance_traveled = -1;

void move_motors(double l, double r, bool reversed) {
    if (reversed) {
        left_motors.move_voltage(-l);
        right_motors.move_voltage(-r);
    } else {
        left_motors.move_voltage(l);
        right_motors.move_voltage(r);
    }
}

void stop_motors() {
    left_motors.move_voltage(0);
    right_motors.move_voltage(0);
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
    } while (std::hypot(getPose().x - target.x, getPose().y - target.y) > threshold);
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

void cancel_all_motions() {
    motion_running = false;
    motion_queued = false;
    pros::delay(10);
}

void turn_heading(double target, double timeout, bool reverse, bool async, double cutoff) {

    request_motion_start();
    if (!motion_running) return;

    if (async) {
        pros::Task([&]() { turn_heading(target, timeout, reverse, false, cutoff); });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;

    if (reverse) target = wrap_angle_180(target + 180);

    // PID tuned in degrees
    PID turn_pid(turn_gains, 2.0, true);  

    ExitCondition turnSmallExit(1.0, 100);  // 1 degree
    ExitCondition turnLargeExit(3.0, 500);  // 3 degrees
    turnSmallExit.reset();
    turnLargeExit.reset();

    while (!timer.isDone() && !turnSmallExit.getExit() && !turnLargeExit.getExit() && motion_running) {
        double current_deg = getPose().theta * (180.0 / M_PI);  // convert to degrees
        double error = wrap_angle_180(target - current_deg);      // error in degrees

        if (cutoff > 0 && fabs(error) < cutoff) break;

        double output = turn_pid.update(error);

        move_motors(output, -output);

        turnSmallExit.update(error);
        turnLargeExit.update(error);

        pros::delay(10);
    }

    stop_motors();

    distance_traveled = -1;
    end_motion();

}

void turn_point(Point target, double timeout, bool reverse, bool async, double cutoff) {
    double angle = atan2(target.y - getPose().y, target.x - getPose().x) * (180 / M_PI);
    turn_heading(angle, timeout, reverse, async, cutoff);
}

void swing_heading(double target, Side locked_side, double timeout, bool reverse, bool async, double cutoff) {
    
    request_motion_start();
    if(!motion_running) return;

    if(async) {
        pros::Task swing_task([&]() mutable {
            swing_heading(target, locked_side, timeout, reverse, false, cutoff);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    if(locked_side == LEFT) left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    else right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    distance_traveled = 0;
    Timer timer(timeout);  
    PID turn_pid(turn_gains);
    if(reverse) target = wrap_angle(target + 180, 360);

    turn_small_exit.reset();
    turn_large_exit.reset();

    while(!turn_small_exit.getExit() && !turn_large_exit.getExit() && !timer.isDone()) {
        double error = target - wrap_angle(getPose().theta * (180 / M_PI), 360);
        if(cutoff > 0 && fabs(error) < cutoff) break;

        double output = turn_pid.update(error);
        turn_small_exit.update(error);
        turn_large_exit.update(error);

        if(locked_side == LEFT) {
            right_motors.move_velocity(output);
        } else {
            left_motors.move_velocity(output);
        }

        pros::delay(10);
    }

    distance_traveled = -1;

    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    end_motion();

}

void swing_point(Point target, Side locked_side, double timeout, bool reverse, bool async, double cutoff) {
    double angle = atan2(target.y - getPose().y, target.x - getPose().x) * (180 / M_PI);
    swing_heading(angle, locked_side, timeout, reverse, async, cutoff);
}

void move_point(Point target, double timeout, bool reverse, bool async, double cutoff) {
    request_motion_start();
    if (!motion_running) return;

    if (async) {
        pros::Task move_task([&]() {
            move_point(target, timeout, reverse, false, cutoff);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    distance_traveled = 0;
    Timer timer(timeout);

    PID drive_pid(drive_gains);
    PID turn_pid(angular_gains, 2.0, true);

    ExitCondition drive_small_exit(1.0, 200);  // 1-inch within for 200ms
    ExitCondition drive_large_exit(3.0, 500); // 5-inches within for 1s

    double prev_drive_out = 0;
    double prev_turn_out = 0;
    const double max_output = 6000;

    while (!drive_small_exit.getExit() && !drive_large_exit.getExit() && !timer.isDone()) {
        Point current(getPose().x, getPose().y);

        // Calculate angle error in compass frame (0 = +Y)
        double dx = target.x - current.x;
        double dy = target.y - current.y;
        double desired_deg = atan2(dx, dy) * (180.0 / M_PI);
        double current_deg = getPose().theta * (180.0 / M_PI);
        double turn_error = wrap_angle_180(desired_deg - current_deg);

        // Distance error
        double drive_error = dist(current.x, current.y, target.x, target.y);

        // Exit if close enough
        if (cutoff > 0 && drive_error < cutoff) break;

        drive_small_exit.update(drive_error);
        drive_large_exit.update(drive_error);

        // PID outputs
        double raw_drive_out = drive_pid.update(drive_error);
        double raw_turn_out = turn_pid.update(turn_error);

        // Clamp turn output
        double turn_out = std::clamp(raw_turn_out, -max_output, max_output);

        // Scale drive by heading alignment (positive only)
        // double alignment = cos(turn_error * M_PI / 180.0);
        // alignment = std::max(0.0, alignment);

        double drive_out = raw_drive_out;

        prev_drive_out = drive_out;
        prev_turn_out = turn_out;

        // Reverse if specified
        if (reverse) drive_out *= -1;

        // Final motor outputs
        move_motors(drive_out + turn_out, drive_out - turn_out);

        pros::delay(10);
    }

    // stop_motors();

    left_motors.move_voltage(0);
    right_motors.move_voltage(0);

    distance_traveled = -1;
    end_motion();
}

void ramsete(std::vector<Waypoint> waypoints, double timeout, bool reverse, bool async, double cutoff,
    double b, double zeta, double time_multi) {

    request_motion_start();
    if (!motion_running) return;

    if (async) {
        pros::Task ramsete_task([&]() {
            ramsete(waypoints, timeout, reverse, false, cutoff, b, zeta, time_multi);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    bool isFinished = false;
    const double dt = 10;

    Timer timer(timeout);

    int waypoint_num = waypoints.size();
    double end_time = waypoints.back().t / time_multi;

    double end_x = waypoints.back().x;
    double end_y = waypoints.back().y;

    int current_waypoint = 0;
    int closest_waypoint = 0;
    double time_passed = 0;
    double time_ahead = 0;

    std::uint32_t now = pros::millis();
    distance_traveled = 0; // Track "progress" like other motions

    ExitCondition drive_small_exit(1.0, 200);  // 1-inch within for 200ms
    ExitCondition drive_large_exit(3.0, 500); // 5-inches within for 1s

    while (time_passed < end_time - time_ahead ||
            (time_passed < end_time - time_ahead - 1000 && !timer.isDone())) {

        double robot_x = getPose().x;
        double robot_y = getPose().y;

        if(cutoff > 0) {
            if (dist(robot_x, robot_y, end_x, end_y) < cutoff) {
                break;
            }
        }

        double robot_h = getPose(true).theta;
        if (reverse) robot_h = fmod(robot_h + M_PI, 2 * M_PI);

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
        double max_val = std::max(std::abs(r_vel), std::abs(l_vel));
        if (max_val > 100) {
            r_vel *= 100 / max_val;
            l_vel *= 100 / max_val;
        }

        double r_volts = voltage_lookup(r_vel);
        double l_volts = voltage_lookup(l_vel);

        move_motors(l_volts, r_volts, reverse);

        distance_traveled += std::abs(error_x) + std::abs(error_y); // crude progress metric

        pros::Task::delay_until(&now, dt);
    }

    end_motion();

    distance_traveled = 0;

    move_point(Point(end_x, end_y), 1000, reverse, async, cutoff);

}