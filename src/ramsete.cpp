#include <string>
#include <vector>
#include "api.h"
#include "mp.h"
#include "odom.h"

void ramsete(std::string path_name, double b, double zeta, double time_multi, double cutoff, double kP, double kD, bool reversed, double t_limit) {
    const double dt = 10;
    const double min_volts = 1.5;

    if (t_limit  == 0) {
        t_limit = 69420;
    }
    double t = pros::millis();

    std::vector<Waypoint> waypoints = fetch_waypoints(path_name);
    int waypoint_num = waypoints.size();
    double end_time = waypoints[waypoint_num - 1].t / time_multi;
    double time_passed = 0;
    double time_ahead = 0;

    double end_x = waypoints[waypoint_num - 1].x;
    double end_y = waypoints[waypoint_num - 1].y;

    int current_waypoint = 0;
    int closest_waypoint = 0;

    std::uint32_t now = pros::millis();

    while (time_passed < end_time - time_ahead && (dist(end_x, end_y, getPose().x, getPose().y) > cutoff || time_passed < end_time - time_ahead - 1000) && pros::c::millis() - t < t_limit ) {
        double robot_x = getPose().x;
        double robot_y = getPose().y;
        double robot_h = getPose().theta;

        if (reversed) {
            robot_h = mod(robot_h + M_PI, 2 * M_PI);
        }

        double closest_waypoint_x = waypoints[closest_waypoint].get_x(); // Find which waypoint the robot is actually on
        double closest_waypoint_y = waypoints[closest_waypoint].get_y();
        double cutoff_dist = dist(robot_x, robot_y, closest_waypoint_x, closest_waypoint_y);
        double search_waypoint_x = waypoints[closest_waypoint + 1].get_x();
        double search_waypoint_y = waypoints[closest_waypoint + 1].get_y();
        double search_dist = dist(robot_x, robot_y, search_waypoint_x, search_waypoint_y);
        while (search_dist < cutoff_dist && closest_waypoint < waypoint_num - 2) {
            closest_waypoint += 1;  
            closest_waypoint_x = waypoints[closest_waypoint].get_x();
            closest_waypoint_y = waypoints[closest_waypoint].get_y();
            cutoff_dist = search_dist;
            search_waypoint_x = waypoints[closest_waypoint + 1].get_x();
            search_waypoint_y = waypoints[closest_waypoint + 1].get_y();
            search_dist = dist(robot_x, robot_y, search_waypoint_x, search_waypoint_y);
        }

        time_passed = pros::c::millis() - t;
        while (time_passed + time_ahead > waypoints[current_waypoint].get_t() / time_multi && current_waypoint < waypoint_num - 1) {
            current_waypoint += 1;
        }

        if (closest_waypoint > current_waypoint) {
            //time_ahead += waypoints[closest_waypoint][5] - time_passed;
            current_waypoint = closest_waypoint;
        }

        double target_x = waypoints[current_waypoint].x;
        double target_y = waypoints[current_waypoint].y;
        double target_h = waypoints[current_waypoint].theta;
        double target_v = waypoints[current_waypoint].linvel;
        double target_w = waypoints[current_waypoint].angvel;

        double error_x = (target_x - robot_x) * std::sin(robot_h) + (target_y - robot_y) * std::cos(robot_h);
        double error_y = -(target_x - robot_x) * std::cos(robot_h) + (target_y - robot_y) * std::sin(robot_h);
        double error_h = roundmod(target_h - (M_PI / 2 - robot_h), 2 * M_PI);

        double k = 2 * zeta * std::sqrt((target_w * target_w) + b * (target_v * target_v));
        double u1 = -k * error_x;
        double u2;
        if (std::abs(error_h) <= 0.01) {
            u2 = -b * target_v * error_y - k * error_h;
        } else {
            u2 = -b * target_v * std::sin(error_h) / error_h * error_y - k * error_h;
        }

        double v = target_v * std::cos(error_h) - u1;
        double w = target_w - u2;

        double percent_v = v / (0.01 * max_rpm / 60 * gear_ratio * circ);
        double percent_w = w * trackwidth / (0.01 * max_rpm / 60 * gear_ratio * circ);

        double r_vel = percent_v + percent_w / 2;
        double l_vel = percent_v - percent_w / 2;

        double max = qol::max({std::abs(r_vel), std::abs(l_vel)});
        if (max > 100) {
            r_vel *= 100 / max;
            l_vel *= 100 / max;
        }

        if (reversed) {
            tchassis::tbh::set_right_target(-1 * l_vel);
            tchassis::tbh::set_left_target(-1 * r_vel);
        } else {
            tchassis::tbh::set_right_target(r_vel);
            tchassis::tbh::set_left_target(l_vel);
        }

        pros::Task::delay_until(&now, dt);
    }

    tchassis::tbh::set_status(false);

    double error;
    if (reversed) {
        error = -1 * dist_to_line(end_x, end_y, tchassis::get_robot_x(), tchassis::get_robot_y(), tchassis::inert::get_heading("deg"));
    } else {
        error = dist_to_line(end_x, end_y, tchassis::get_robot_x(), tchassis::get_robot_y(), tchassis::inert::get_heading("deg"));
    }
    double prev_error = error;
    while (error > 1 && pros::c::millis() - t < t_limit) {
        if (reversed) {
            error = -1 * dist_to_line(end_x, end_y, tchassis::get_robot_x(), tchassis::get_robot_y(), tchassis::inert::get_heading("deg"));
        } else {
            error = dist_to_line(end_x, end_y, tchassis::get_robot_x(), tchassis::get_robot_y(), tchassis::inert::get_heading("deg"));
        }

        double delta_error = error - prev_error;
        double volts = kP * error + kD * delta_error / dt;

        if (std::abs(volts) < min_volts) {
            volts = sign(volts) * min_volts;
        }
        if (std::abs(volts) > 12) {
            volts = sign(volts) * 12;
        }

        if (reversed) {
            tchassis::left::move_voltage(volts * -1000);
            tchassis::right::move_voltage(volts * -1000);
        } else {
            tchassis::left::move_voltage(volts * 1000);
            tchassis::right::move_voltage(volts * 1000);
        }

        prev_error = error;

        pros::Task::delay_until(&now, dt);
    }
    tchassis::brake();
}