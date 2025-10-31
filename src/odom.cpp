#include "odom.h"
#include "mcl.h"
#include "config.h"

#define IMU_CW_DRIFT 1.0
#define IMU_CCW_DRIFT 1.0

bool wheel_tracking_enabled = true;

void set_wheel_tracking(bool enabled) {
    wheel_tracking_enabled = enabled;
}

bool get_wheel_tracking() {
    return wheel_tracking_enabled;
}

Pose robot_pose = Pose(0, 0, 0);
Pose robot_speed = Pose(0, 0, 0);

Pose get_pose(PoseSettings settings) {
    double theta = robot_pose.theta;
    if(settings.degrees) theta *= 180.0 / M_PI;
    if(settings.standard) return Pose(robot_pose.x, robot_pose.y, M_PI / 2 - theta);
    return Pose(robot_pose.x, robot_pose.y, theta);
}

Pose get_speed() {
    return robot_speed;
}

void set_pose(Pose new_pose) {
    robot_pose = new_pose;
}

double prev_left_raw = 0;
double prev_right_raw = 0;
double prev_theta_raw = 0;

void start_odom(Pose initial_pose) {
    robot_pose = initial_pose;
    robot_speed = Pose(0, 0, 0);
    prev_left_raw = left_motors.get_average_position();   
    prev_right_raw = right_motors.get_average_position();
    prev_theta_raw = imu.get_rotation() * M_PI / 180.0;
}

void update_odom() {

    double left_raw = left_motors.get_average_position();   
    double right_raw = right_motors.get_average_position();

    double theta_raw = imu.get_rotation() * M_PI / 180.0;
    // theta_raw *= (theta_raw > prev_theta_raw) ? IMU_CW_DRIFT : IMU_CCW_DRIFT;

    double left_delta = left_raw - prev_left_raw;
    double right_delta = right_raw - prev_right_raw;

    double theta_delta = theta_raw - prev_theta_raw;

    if(!wheel_tracking_enabled) {
        robot_speed.x = 0;
        robot_speed.y = 0;
        robot_pose.theta += theta_delta;
        prev_theta_raw = theta_raw;
        return;
    }

    double heading = robot_pose.theta + theta_delta;
    double avg_heading = robot_pose.theta + theta_delta / 2;

    double left_delta_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;
    double right_delta_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0;

    double mid_delta_in = (left_delta_in + right_delta_in) / 2.0;

    double local_y = 0;

    if(std::fabs(theta_delta) < 1e-6) local_y = mid_delta_in; 
    else local_y = 2 * sin(theta_delta / 2.0) * (mid_delta_in / theta_delta);

    prev_left_raw = left_raw;
    prev_right_raw = right_raw;
    prev_theta_raw = theta_raw;
    
    robot_speed.x = local_y * sin(avg_heading);
    robot_speed.y = local_y * cos(avg_heading);
    robot_speed.theta = theta_delta;

    robot_pose.x += robot_speed.x;
    robot_pose.y += robot_speed.y;
    robot_pose.theta = heading;

}