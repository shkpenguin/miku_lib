// #include "main.h"
// #include "pros/llemu.hpp"
// #include <algorithm>
// #include <map>
// #include <vector>
// #include <cmath>

// namespace tchassis::mp::catmull_rom {

// struct Waypoint {
//     double x, y;
//     double theta;
//     double linvel;  // in/sec
//     double angvel;  // rad/sec
//     double t;       // ms
// };

// static std::map<std::string, std::vector<std::vector<double>>> paths;
// static std::map<std::string, std::vector<Waypoint>> computed_paths;

// void fill_paths() {
//     paths["skull"] = {
//         {-48, -12, 0}, {-48, 0, 20}, {-48, 15, 20}, {-41, 20, 25},
//         {-27, 23, 20}, {-22, 31, 15}, {-17, 34, 40}, {-10, 37, 25},
//         {0, 48, 23}, {0, 53, 20}, {-3, 54, 18}, {-8, 53, 18},
//         {-16, 51, 20}, {-26, 50, 20}, {-36, 50, 20}, {-60, 50, 16},
//         {-58, 53, 15}, {-48, 60, 18}, {-36, 72, 0}
//     };
//     // Add other paths...
// }

// std::vector<double> get_derivatives(
//     const std::vector<std::vector<double>>& P,
//     const std::vector<double>& T, double t)
// {
//     // ... same math as before, just clean formatting ...
// }

// std::vector<Waypoint> calculate_waypoints(
//     const std::vector<std::vector<double>>& control_points, double spacing)
// {
//     std::vector<Waypoint> waypoints;
//     double current_time = 0;
//     double prev_x = control_points[1][0];
//     double prev_y = control_points[1][1];

//     for (size_t i = 0; i + 3 < control_points.size(); i++) {
//         int n = dist(control_points[i+1][0], control_points[i+1][1],
//                      control_points[i+2][0], control_points[i+2][1]) / spacing;

//         std::vector<std::vector<double>> P = {
//             control_points[i], control_points[i+1],
//             control_points[i+2], control_points[i+3]
//         };

//         std::vector<double> T(4, 0);
//         for (int j = 1; j < 4; j++) {
//             T[j] = pow(square(P[j][0] - P[j-1][0]) +
//                        square(P[j][1] - P[j-1][1]), 0.25) + T[j-1];
//         }

//         for (int step = 0; step <= n; ++step) {
//             double t = T[1] + (T[2] - T[1]) * step / n;
//             auto d = get_derivatives(P, T, t);

//             Waypoint point;
//             point.x = d[0];
//             point.y = d[1];
//             point.theta = std::atan2(d[3], d[2]);
//             // velocity / angvel math same as before...
//             current_time += dist(prev_x, prev_y, point.x, point.y) / point.linvel * 1000;
//             point.t = current_time;

//             waypoints.push_back(point);
//             prev_x = point.x;
//             prev_y = point.y;
//         }
//     }

//     return waypoints;
// }

// void precompute_paths() {
//     fill_paths();
//     for (const auto& [name, cps] : paths) {
//         computed_paths[name] = calculate_waypoints(cps, 0.5);
//     }
// }

// const std::vector<Waypoint>& fetch_waypoints(const std::string& name) {
//     return computed_paths[name];
// }

// } // namespace tchassis::mp::catmull_rom
