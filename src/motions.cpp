#include "motions.h"
#include "util.h"
#include "odom.h"
#include "config.h"
#include "timer.h"

pros::Mutex motion_mutex;

bool motion_running = false;
bool motion_queued = false;
double distance_traveled = -1;

void wait_until_done() {
    do {
        pros::delay(10);
    } while(distance_traveled != -1);
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

void turn_heading(double target, bool reverse, double timeout, bool async = false, double cutoff = -1.0) {

    request_motion_start();
    if(!motion_running) return;

    if(async) {
        pros::Task turn_task([=]() {
            turn_heading(target, reverse, timeout, false, cutoff);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    Timer timer(timeout);
    distance_traveled = 0;
    PID turn_pid(turn_gains);
    if(reverse) target = wrap_angle(target + 180, 360);
    else target = wrap_angle(target, 360);

    turn_small_exit.reset();
    turn_large_exit.reset();

    while(!turn_small_exit.getExit() && !turn_large_exit.getExit() && !timer.isDone()) {
        double error = target - wrap_angle(getPose().theta * (180 / M_PI), 360);
        if(cutoff > 0 && fabs(error) < cutoff) break;

        double output = turn_pid.update(error);
        turn_small_exit.update(error);
        turn_large_exit.update(error);

        move_motors(output, -output);

        pros::delay(10);
    }

    end_motion();

}

void turn_point(Point target, bool reverse, double timeout, bool async = false, double cutoff = -1.0) {
    double angle = atan2(target.y - getPose().y, target.x - getPose().x) * (180 / M_PI);
    turn_heading(angle, reverse, timeout, async, cutoff);
}

void swing_heading(double target, Side locked_side, bool reverse, double timeout, bool async = false, double cutoff = -1.0) {
    
    request_motion_start();
    if(!motion_running) return;

    if(async) {
        pros::Task swing_task([=]() mutable {
            swing_heading(target, locked_side, reverse, timeout, false, cutoff);
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

void swing_point(Point target, Side locked_side, bool reverse, double timeout, bool async = false, double cutoff = -1.0) {
    double angle = atan2(target.y - getPose().y, target.x - getPose().x) * (180 / M_PI);
    swing_heading(angle, locked_side, reverse, timeout, async, cutoff);
}

void move_point(Point target, bool reverse, double timeout, bool async = false, double cutoff = -1.0) {

    request_motion_start();
    if(!motion_running) return;

    if(async) {
        pros::Task move_task([=]() mutable {
            move_point(target, reverse, timeout, false, cutoff);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    distance_traveled = 0;
    Timer timer(timeout);  
    PID drive_pid(drive_gains);
    PID turn_pid(turn_gains);

    drive_small_exit.reset();
    drive_large_exit.reset();

    while(!drive_small_exit.getExit() && !drive_large_exit.getExit() && !timer.isDone()) {
        Point current = Point(getPose().x, getPose().y);

        double turn_error = wrap_angle(atan2(target.y - current.y, target.x - current.x) * (180 / M_PI), 360) - wrap_angle(getPose().theta * (180 / M_PI), 360);
        double turn_output = turn_pid.update(turn_error);

        double drive_error = dist(current.x, current.y, target.x, target.y);
        if(cutoff > 0 && drive_error < cutoff) break;

        double drive_scale = std::max(0.0, cos(turn_error * M_PI / 180));
        double drive_output = drive_pid.update(drive_error) * drive_scale;

        move_motors(drive_output + turn_output, drive_output - turn_output);
    }

    distance_traveled = -1;

    end_motion();

}

void ramsete(std::vector<Waypoint> waypoints, double b, double zeta, double time_multi,
             double cutoff, double kP, double kD, bool reversed, double t_limit,
             bool async = false) {

    request_motion_start();
    if (!motion_running) return;

    if (async) {
        pros::Task ramsete_task([=]() mutable {
            ramsete(waypoints, b, zeta, time_multi, cutoff, kP, kD, reversed, t_limit, false);
        });
        end_motion();
        pros::delay(10);
        return;
    }

    const double dt = 10;

    if (t_limit == 0) t_limit = 69420;
    double t_start = pros::millis();

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

    while (time_passed < end_time - time_ahead &&
           (dist(end_x, end_y, getPose().x, getPose().y) > cutoff ||
            time_passed < end_time - time_ahead - 1000) &&
           pros::c::millis() - t_start < t_limit) {

        double robot_x = getPose().x;
        double robot_y = getPose().y;
        double robot_h = getPose(true).theta + (reversed ? M_PI : 0);

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
        time_passed = pros::c::millis() - t_start;
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

        double error_x = (target_x - robot_x) * std::sin(robot_h) + (target_y - robot_y) * std::cos(robot_h);
        double error_y = -(target_x - robot_x) * std::cos(robot_h) + (target_y - robot_y) * std::sin(robot_h);
        double error_h = angle_error(target_h, M_PI / 2 - robot_h);

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

        move_motors(r_vel, l_vel);

        distance_traveled += std::abs(error_x) + std::abs(error_y); // crude progress metric

        pros::Task::delay_until(&now, dt);
    }

    // Final projection loop
    auto get_error = [&]() {
        double e = project(end_x, end_y, getPose().x, getPose().y, getPose(true).theta);
        return reversed ? -e : e;
    };

    double error = get_error();
    double prev_error = error;

    while (error > 1 && pros::c::millis() - t_start < t_limit) {
        error = get_error();
        double delta_error = error - prev_error;
        double volts = clamp(kP * error + kD * delta_error / dt, 1.5, 12.0);
        move_motors(volts, volts);
        prev_error = error;
        pros::Task::delay_until(&now, dt);
    }

    end_motion();
}