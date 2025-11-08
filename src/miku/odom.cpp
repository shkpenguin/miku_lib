#include "mcl.h"
#include "chassis.h"
#include "config.h"

#define IMU_CW_DRIFT 1.0
#define IMU_CCW_DRIFT 1.0

double prev_left_raw = 0;
double prev_right_raw = 0;
compass_degrees prev_theta_raw = 0;

inline void miku::Chassis::reset(Pose initial_pose) {
    set_pose(initial_pose);
    pose_delta = Pose(0, 0, 0);
    prev_left_raw = left_motors->get_average_position();   
    prev_right_raw = right_motors->get_average_position();
    prev_theta_raw = imu.get_rotation() * M_PI / 180.0;
}

void miku::Chassis::update_odometry() {

    double left_raw = left_motors->get_average_position();   
    double right_raw = right_motors->get_average_position();

    compass_degrees theta_raw = imu.get_rotation() * M_PI / 180.0;
    theta_raw *= (theta_raw > prev_theta_raw) ? IMU_CW_DRIFT : IMU_CCW_DRIFT;

    double left_delta = left_raw - prev_left_raw;
    double right_delta = right_raw - prev_right_raw;

    standard_radians theta_delta = standard_radians(theta_raw - prev_theta_raw);

    // if(!wheel_tracking_enabled) {
    //     robot_speed.x = 0;
    //     robot_speed.y = 0;
    //     robot_pose.theta += theta_delta;
    //     prev_theta_raw = theta_raw;
    //     return;
    // }

    double heading = pose.theta + theta_delta;
    double avg_heading = pose.theta + theta_delta / 2;

    double left_delta_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;
    double right_delta_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;

    double mid_delta_in = (left_delta_in + right_delta_in) / 2.0;

    double local_y = 0;

    if(std::fabs(theta_delta) < 1e-6) local_y = mid_delta_in; 
    else local_y = 2 * sin(theta_delta / 2.0) * (mid_delta_in / theta_delta);

    prev_left_raw = left_raw;
    prev_right_raw = right_raw;
    prev_theta_raw = theta_raw;
    
    pose_delta.x = local_y * sin(avg_heading);
    pose_delta.y = local_y * cos(avg_heading);
    pose_delta.theta = theta_delta;

    pose.x += pose_delta.x;
    pose.y += pose_delta.y;
    pose.theta = heading;

}