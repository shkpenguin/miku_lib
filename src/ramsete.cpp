#include <string>
#include <vector>
#include "api.h"
#include "mp.h"
#include "odom.h"
#include "misc.h"
#include "config.h"
#include "util.h"
#include "motions.h"

void ramsete(std::vector<Waypoint> waypoints, double b, double zeta, double time_multi,
             double cutoff, double kP, double kD, bool reversed, double t_limit) {

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
}