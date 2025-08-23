#include "odom.h"
#include "mcl.h"
#include "config.h"

#define dt 0.01

Pose robot_pose = Pose(0, 0, 0);
Pose robot_speed = Pose(0, 0, 0);

Pose getPose(bool standard) {
    if(standard) return Pose(robot_pose.x, robot_pose.y, M_PI/2 - robot_pose.theta);
    return robot_pose;
}

Pose getSpeed() {
    return robot_speed;
}

Pose setPose(Pose new_pose) {
    robot_pose = new_pose;
    return robot_pose;
}

double prev_left_raw = 0;
double prev_right_raw = 0;
double prev_theta_raw = 0;

void update() {

    double left_raw = left_motors.get_position();   
    double right_raw = right_motors.get_position();

    double theta_raw = imu.get_rotation() * M_PI / 180.0;

    double left_delta = left_raw - prev_left_raw;
    double right_delta = right_raw - prev_right_raw;

    double theta_delta = theta_raw - prev_theta_raw;
    double avg_heading = robot_pose.theta + theta_delta / 2;

    double left_delta_in = left_delta * WHEEL_DIAMETER * M_PI / 360.0;
    double right_delta_in = right_delta * WHEEL_DIAMETER * M_PI / 360.0;

    double mid_delta_in = (left_delta_in + right_delta_in) / 2.0;

    double local_y = 0;

    if(std::fabs(theta_delta) < 1e-6) local_y = mid_delta_in; 
    else local_y = 2 * sin(theta_delta / 2.0) * (mid_delta_in / theta_delta);

    prev_left_raw = left_raw;
    prev_right_raw = right_raw;
    prev_theta_raw = theta_raw;
    
    robot_speed.x = local_y * sin(avg_heading);
    robot_speed.y = local_y * cos(avg_heading);

    robot_pose.x += robot_speed.x;
    robot_pose.y += robot_speed.y;
    robot_pose.theta = theta_raw;

}

void init_odom(Pose initial_pose) {
    setPose(initial_pose);
    initialize_particles();
}

void track_odom() {
    while(true) {
        update();
        update_particles();

        Pose estimate = get_pose_estimate();
        setPose(Pose(estimate.x, estimate.y, robot_pose.theta));
        
        resampleParticles();
        pros::delay(20); // Update every 20 ms
    }
};