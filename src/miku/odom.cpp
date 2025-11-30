#include "miku/miku-api.h"

#define IMU_CW_DRIFT 1.0f
#define IMU_CCW_DRIFT 1.0f

float prev_left_raw = 0;
float prev_right_raw = 0;
compass_degrees prev_theta_raw = 0;

void miku::Chassis::calibrate() {
    prev_left_raw = left_motors->get_average_position();   
    prev_right_raw = right_motors->get_average_position();
    prev_theta_raw = compass_degrees(imu->get_rotation());
}

Pose miku::Chassis::compute_odometry_delta() {

    float left_raw = left_motors->get_average_position();
    float right_raw = right_motors->get_average_position();

    // read imu and convert to standard radians immediately
    compass_degrees theta_raw = compass_degrees(imu->get_rotation());

    float left_delta = left_raw - prev_left_raw;
    float right_delta = right_raw - prev_right_raw;

    // compute signed shortest difference in radians
    float theta_delta = -1.0f * ((theta_raw - prev_theta_raw).radians());

    // update heading variables
    standard_radians heading_new = pose.theta + theta_delta;
    standard_radians avg_heading = pose.theta + theta_delta / 2.0f;

    float left_delta_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;
    float right_delta_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;

    float mid_delta_in = (left_delta_in + right_delta_in) / 2.0f;

    float local_y = 0;

    if (std::fabs(float(theta_delta)) < 1e-6f) local_y = mid_delta_in;
    else local_y = 2.0f * sin(float(theta_delta) / 2.0f) * (mid_delta_in / float(theta_delta));

    prev_left_raw = left_raw;
    prev_right_raw = right_raw;
    prev_theta_raw = theta_raw;

    Pose pose_delta;
    pose_delta.x = local_y * cos(float(avg_heading));
    pose_delta.y = local_y * sin(float(avg_heading));
    pose_delta.theta = theta_delta;

    return pose_delta;

}

void miku::Chassis::update_position() {
    
    Pose delta = compute_odometry_delta();
    set_heading(pose.theta + delta.theta);

    pf->update_previous_belief(delta);
    pf->update_particle_weights();

    Point belief = pf->get_current_belief();
    set_position(belief);

    pf->resample_particles();

}