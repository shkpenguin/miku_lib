#include "motions.h"

Ramsete::Ramsete(std::vector<Waypoint> wps, double timeout, RamseteParams p)
: waypoints(wps), timeout(timeout), params(p) {
if(p.zeta < 0) zeta = 0.7;
if(p.b < 0) b = 0.003;
if(p.time_multi < 0) time_multi = 1.2;
}
    
void Ramsete::start() {
    done = false;
    timer.set(timeout);
    timer.reset();
    drive_small_exit.reset();
    drive_large_exit.reset();

    current_waypoint = 0;
    closest_waypoint = 0;
    time_passed = 0;
    time_ahead = 0;
}

void Ramsete::update() {
    Pose current = get_pose();

    double end_x = waypoints.back().x;
    double end_y = waypoints.back().y;
    double end_time = waypoints.back().t / time_multi;

    double robot_x = current.x;
    double robot_y = current.y;
    double robot_h = current.theta;
    if (params.reverse) robot_h = fmod(robot_h + M_PI, 2 * M_PI);

    // Distance to end
    double distance_to_end = dist(end_x, end_y, robot_x, robot_y);
    if (distance_to_end < params.cutoff || distance_to_end < params.end_cutoff) {
        done = true;
        return;
    }

    // Update closest waypoint
    double cutoff_dist = dist(robot_x, robot_y, waypoints[closest_waypoint].x, waypoints[closest_waypoint].y);
    while (closest_waypoint < (int)waypoints.size() - 2) {
        double search_dist = dist(robot_x, robot_y, waypoints[closest_waypoint + 1].x,
                                    waypoints[closest_waypoint + 1].y);
        if (search_dist >= cutoff_dist) break;
        closest_waypoint++;
        cutoff_dist = search_dist;
    }

    // Update current waypoint by time
    time_passed = timer.get_time_passed();
    while (current_waypoint < (int)waypoints.size() - 1 &&
            time_passed + time_ahead > waypoints[current_waypoint].t / time_multi) {
        current_waypoint++;
    }
    if (closest_waypoint > current_waypoint) current_waypoint = closest_waypoint;

    const auto& wp = waypoints[current_waypoint];
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

    double r_volts = params.reverse ? drive_lut.get_value(-l_vel) : drive_lut.get_value(r_vel);
    double l_volts = params.reverse ? drive_lut.get_value(-r_vel) : drive_lut.get_value(l_vel);

    move_motors(l_volts, r_volts);

    if(drive_small_exit.get_exit() || drive_large_exit.get_exit() || timer.is_done()) {
        done = true;

        // Schedule a MovePoint after finishing
        auto next_motion = std::make_shared<MovePoint>(
            Point{end_x, end_y},
            timer.get_time_left(),
            MovePointParams(params.reverse, params.cutoff, params.max_vel * 120, params.min_vel * 120)
        );
        queue_after_current(next_motion);
    }
}

bool Ramsete::is_done() {
    return done;
}