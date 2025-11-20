#include "miku/motions.h"
#include "system.h"
#include "miku/devices/chassis.h"
#include "config.h"

Ramsete::Ramsete(std::vector<Waypoint> wps, double timeout, RamseteParams p)
: waypoints(wps), timeout(timeout), params(p) {
    if(p.zeta < 0) zeta = 0.7;
    if(p.b < 0) b = 0.01;
    if(p.time_multi < 0) time_multi = 1.2;
}
    
void Ramsete::start() {
    done = false;
    timer.set(timeout);
    timer.reset();

    current_waypoint = 0;
    closest_waypoint = 0;
    time_passed = 0;
    time_ahead = 0;
}

void Ramsete::update() {
    Pose current = Miku.get_pose();

    double end_x = waypoints.back().x;
    double end_y = waypoints.back().y;
    double end_time = waypoints.back().t / time_multi;

    double robot_x = current.x;
    double robot_y = current.y;
    standard_radians robot_h = current.theta;
    if (params.reverse) robot_h = fmod(robot_h + M_PI, 2 * M_PI);

    // Distance to end
    double distance_to_end = current.distance_to(Point(end_x, end_y));
    if (distance_to_end < params.cutoff || distance_to_end < params.end_cutoff) {
        done = true;
        return;
    }

    // Update closest waypoint
    double cutoff_dist = current.distance_to(Point(waypoints[closest_waypoint].x, waypoints[closest_waypoint].y));
    while (closest_waypoint < (int)waypoints.size() - 2) {
        double search_dist = current.distance_to(Point(waypoints[closest_waypoint + 1].x,
                                    waypoints[closest_waypoint + 1].y));
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
    standard_radians target_h = wp.theta;
    double target_v = wp.linvel, target_w = wp.angvel;

    double dx = target_x - robot_x;
    double dy = target_y - robot_y;

    double error_x =  std::cos(robot_h) * dx + std::sin(robot_h) * dy;
    double error_y = -std::sin(robot_h) * dx + std::cos(robot_h) * dy;
    standard_radians error_h = (target_h - robot_h).norm();

    double k = 2 * zeta * std::sqrt(target_w * target_w + b * target_v * target_v);
    double u1 = -k * error_x;
    double u2 = (std::abs(error_h) <= 0.01) ? (-b * target_v * error_y - k * error_h)
                                            : (-b * target_v * std::sin(error_h) / error_h * error_y - k * error_h);
    double v = target_v * std::cos(error_h) - u1;
    double w = target_w - u2;

    double l_vel = v - w * TRACK_WIDTH / 2; // in/s
    double r_vel = v + w * TRACK_WIDTH / 2; // in/s

    double l_rpm = (l_vel * 60.0) / (CIRC * GEAR_RATIO);
    double r_rpm = (r_vel * 60.0) / (CIRC * GEAR_RATIO);

    Miku.move_velocity(l_rpm, r_rpm);

    if(timer.is_done() || time_passed >= end_time) {
        auto settle_motion = new MovePoint(
            Point(end_x, end_y),
            timer.get_time_left(),
            MovePointParams{
                params.reverse,
                -1.0,
                12000.0,
                1000.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0
            }
        );

        if(!done) queue_after_current(settle_motion);
        Miku.stop();
        done = true;

    }
}

bool Ramsete::is_done() {
    return done;
}