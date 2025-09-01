#include "mp.h"
#include <cmath>
#include <algorithm>

double get_t_param(std::vector<ControlPoint> P) {
    double dx = P[2].x - P[0].x;
    double dy = P[2].y - P[0].y;
    double k3 = dx * dx + dy * dy;
    double k2 = 3 * (dx * (P[0].x - P[1].x) + dy * (P[0].y - P[1].y));
    double k1 = (3 * P[0].x - 2 * P[1].x - P[2].x) * (P[0].x - P[1].x) +
                (3 * P[0].y - 2 * P[1].y - P[2].y) * (P[0].y - P[1].y);
    double k0 = -((P[0].x - P[1].x) * (P[0].x - P[1].x) +
                  (P[0].y - P[1].y) * (P[0].y - P[1].y));

    double estimate = 0.5;
    double bucket_size = 0.5;
    for (int i = 0; i < 20; i++) {
        double value = k3 * estimate * estimate * estimate +
                       k2 * estimate * estimate +
                       k1 * estimate + k0;
        estimate += (value < 0 ? 1 : -1) * bucket_size / 2;
        bucket_size /= 2;
    }
    return estimate;
}

std::vector<double> get_derivatives_quadratic_bezier(const std::vector<ControlPoint>& P, double t) {
    double u = 1 - t;
    double u2 = u * u;
    double t2 = t * t;

    double x = u2 * P[0].x + 2 * u * t * P[1].x + t2 * P[2].x;
    double y = u2 * P[0].y + 2 * u * t * P[1].y + t2 * P[2].y;

    double dx = 2 * u * (P[1].x - P[0].x) + 2 * t * (P[2].x - P[1].x);
    double dy = 2 * u * (P[1].y - P[0].y) + 2 * t * (P[2].y - P[1].y);

    double ddx = 2 * (P[2].x - 2 * P[1].x + P[0].x);
    double ddy = 2 * (P[2].y - 2 * P[1].y + P[0].y);

    return {x, y, dx, dy, ddx, ddy};
}

std::vector<double> get_derivatives(
    std::vector<ControlPoint>& P,
    std::vector<ControlPoint>& MCP,
    ControlPoint& TP,
    double t
) {
    const double pi_2 = M_PI / 2;
    double ct = std::cos(t);
    double st = std::sin(t);
    double ct2 = ct * ct;
    double st2 = st * st;
    double two_st_ct = 2 * st * ct;

    // First and second Bezier segments
    auto F1 = get_derivatives_quadratic_bezier({P[0], MCP[0], P[2]}, TP.x + (1 - TP.x) * t / pi_2);
    auto F2 = get_derivatives_quadratic_bezier({P[1], MCP[1], P[3]}, TP.y * t / pi_2);

    double scale1 = (1 - TP.x) / pi_2;
    double scale2 = TP.y / pi_2;

    for (int i = 2; i < 6; i++) {
        F1[i] *= (i < 4 ? scale1 : scale1 * scale1);
        F2[i] *= (i < 4 ? scale2 : scale2 * scale2);
    }

    // Final combined derivatives
    double x   = ct2 * F1[0] + st2 * F2[0];
    double y   = ct2 * F1[1] + st2 * F2[1];
    double dx  = two_st_ct * (F2[0] - F1[0]) + ct2 * F1[2] + st2 * F2[2];
    double dy  = two_st_ct * (F2[1] - F1[1]) + ct2 * F1[3] + st2 * F2[3];
    double ddx = 2 * (ct2 - st2) * (F2[0] - F1[0]) + 4 * st * ct * (F2[2] - F1[2]) + ct2 * F1[4] + st2 * F2[4];
    double ddy = 2 * (ct2 - st2) * (F2[1] - F1[1]) + 4 * st * ct * (F2[3] - F1[3]) + ct2 * F1[5] + st2 * F2[5];

    return {x, y, dx, dy, ddx, ddy};
}

void BezierPath::calculate_waypoints() {
    double spacing = 0.1;

    std::vector<Waypoint> _waypoints;
    int points = control_points.size();

    double current_time = 0;
    double prev_x = control_points[0].x, prev_y = control_points[0].y;
    double prev_t = 0, prev_dx = 0, prev_dy = 0, prev_ddx = 0, prev_ddy = 0;
    bool first_loop = true;

    for (int i = 0; i <= points - 4; i++) {

        std::vector<ControlPoint> P;

        for(int j = 0; j < 4; ++j) {
            P.push_back(control_points[i + j]);
        }

        int n = std::hypot(P[2].x - P[1].x, P[2].y - P[1].y) / spacing;

        std::vector<ControlPoint> MCP(2, {0, 0});
        ControlPoint TP;

        double t_param = get_t_param({P[0], P[1], P[2]});
        MCP[0].x = (P[1].x - (1 - t_param) * (1 - t_param) * P[0].x - t_param * t_param * P[2].x) /
                (2 * t_param * (1 - t_param));
        MCP[0].y = (P[1].y - (1 - t_param) * (1 - t_param) * P[0].y - t_param * t_param * P[2].y) /
                (2 * t_param * (1 - t_param));
        TP.x = t_param;
    
        t_param = get_t_param({P[1], P[2], P[3]});
        MCP[1].x = (P[2].x - (1 - t_param) * (1 - t_param) * P[1].x - t_param * t_param * P[3].x) /
                (2 * t_param * (1 - t_param));
        MCP[1].y = (P[2].y - (1 - t_param) * (1 - t_param) * P[1].y - t_param * t_param * P[3].y) /
                (2 * t_param * (1 - t_param));
        TP.y = t_param;

        for (double t = 0; t < M_PI / 2; t += M_PI / 2 / n + 0.001) {
            auto derivatives = get_derivatives(P, MCP, TP, t);
            Waypoint point;
            point.x = derivatives[0];
            point.y = derivatives[1];
            point.theta = std::atan2(derivatives[3], derivatives[2]);

            double linvel = (1 - t / (M_PI / 2)) * P[1].velocity + (t / (M_PI / 2)) * P[2].velocity;
            double in_per_sec = linvel * 0.01 * MAX_RPM / 60 * GEAR_RATIO * CIRC;

            double curvature = 0, rad_per_sec = 0;
            if (derivatives[2] != 0 || derivatives[3] != 0) {
                curvature = (derivatives[2] * derivatives[5] - derivatives[3] * derivatives[4]) /
                            std::pow(derivatives[2]*derivatives[2] + derivatives[3]*derivatives[3], 1.5);
                rad_per_sec = linvel * curvature;
            }

            double angvel = rad_per_sec * TRACK_WIDTH / (0.01 * MAX_RPM / 60 * GEAR_RATIO * CIRC);

            double r_vel = linvel + angvel / 2;
            double l_vel = linvel - angvel / 2;
            double max_val = std::max(std::abs(r_vel), std::abs(l_vel));

            if (max_val > 100) {
                double scale = 100 / max_val;
                r_vel *= scale; l_vel *= scale;
                linvel *= scale; angvel *= scale;
                in_per_sec *= scale; rad_per_sec *= scale;
            }

            point.linvel = in_per_sec;
            point.angvel = rad_per_sec;

            if(in_per_sec > 0) current_time += std::hypot(prev_x - point.x, prev_y - point.y) / in_per_sec * 1000;
            else current_time = 0;
            point.t = current_time;

            point.dx = point.linvel * std::cos(point.theta);
            point.dy = point.linvel * std::sin(point.theta);

            if (first_loop) {
                prev_dx = point.dx;
                prev_dy = point.dy;
                prev_t = -10;
                first_loop = false;
            }

            if (std::abs(point.t - prev_t) >= 0.001) {
                double dt = (point.t - prev_t) / 1000;
                point.ddx = (point.dx - prev_dx) / dt;
                point.ddy = (point.dy - prev_dy) / dt;
            } else {
                point.ddx = prev_ddx;
                point.ddy = prev_ddy;
            }

            prev_x = point.x; prev_y = point.y;
            prev_dx = point.dx; prev_dy = point.dy;
            prev_ddx = point.ddx; prev_ddy = point.ddy;
            prev_t = point.t;

            _waypoints.push_back(point);
        }
    }

    waypoints = {_waypoints};
}