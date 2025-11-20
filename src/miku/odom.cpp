#include "miku/miku-api.h"

#define IMU_CW_DRIFT 1.0
#define IMU_CCW_DRIFT 1.0

double prev_left_raw = 0;
double prev_right_raw = 0;
compass_degrees prev_theta_raw = 0;

void miku::Chassis::reset(Pose initial_pose) {
    set_pose(initial_pose);
    pose_delta = Pose(0, 0, 0);
    prev_left_raw = left_motors->get_average_position();   
    prev_right_raw = right_motors->get_average_position();
    prev_theta_raw = compass_degrees(imu.get_rotation());
}

void miku::Chassis::update_odometry() {

    double left_raw = left_motors->get_average_position();
    double right_raw = right_motors->get_average_position();

    // read imu and convert to standard radians immediately
    compass_degrees theta_raw = compass_degrees(imu.get_rotation());

    double left_delta = left_raw - prev_left_raw;
    double right_delta = right_raw - prev_right_raw;

    // compute signed shortest difference in radians
    double theta_delta = -1 * ((theta_raw - prev_theta_raw).radians());

    // update heading variables
    standard_radians heading_new = pose.theta + theta_delta;
    standard_radians avg_heading = pose.theta + theta_delta / 2.0;

    double left_delta_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;
    double right_delta_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;

    double mid_delta_in = (left_delta_in + right_delta_in) / 2.0;

    double local_y = 0;

    if (std::fabs(double(theta_delta)) < 1e-6) local_y = mid_delta_in;
    else local_y = 2.0 * sin(double(theta_delta) / 2.0) * (mid_delta_in / double(theta_delta));

    prev_left_raw = left_raw;
    prev_right_raw = right_raw;
    prev_theta_raw = theta_raw;

    pose_delta.x = local_y * cos(double(avg_heading));
    pose_delta.y = local_y * sin(double(avg_heading));
    pose_delta.theta = theta_delta;

    pose.x += pose_delta.x;
    pose.y += pose_delta.y;
    pose.theta = heading_new;

    update_particles();
    log_mcl();

    Miku.set_position(get_position_estimate());
    resample_particles();

}