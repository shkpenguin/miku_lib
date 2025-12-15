#pragma once

#include "pros/motors.hpp"
#include "miku/devices/motor.h"
#include "miku/geometry.h"
#include "miku/mcl.h"
#include <memory>

inline pros::Mutex pose_mutex;

namespace miku {

class Chassis {
public:
    Chassis();
    Chassis(std::shared_ptr<MotorGroup> left, std::shared_ptr<MotorGroup> right, std::shared_ptr<pros::Imu> imu, std::shared_ptr<ParticleFilter> pf)
        : left_motors(left), right_motors(right), imu(imu), pf(pf) {}

    void move(float left_speed, float right_speed) {
        left_motors->move(left_speed);
        right_motors->move(right_speed);
    }
    void move_voltage(float left_speed, float right_speed) {
        left_motors->move_voltage(left_speed);
        right_motors->move_voltage(right_speed);
    }
    void move_velocity(float left_velocity, float right_velocity) {
        left_motors->move_velocity(left_velocity);
        right_motors->move_velocity(right_velocity);
    }
    void stop() {
        left_motors->move_voltage(0);
        right_motors->move_voltage(0);
    }
    void set_brake_mode(pros::motor_brake_mode_e mode) {
        left_motors->set_brake_mode(mode);
        right_motors->set_brake_mode(mode);
    }
    void set_use_particle_filtering(bool use_pf) {
        use_particle_filtering = use_pf;
    }
 
    inline Pose get_pose() { return pose; };
    inline Point get_position() { return Point(pose.x, pose.y); };
    inline float get_x() { return pose.x; };
    inline float get_y() { return pose.y; };
    inline standard_radians get_heading() { return pose.theta; };
    inline void set_x(float x) {
        pose_mutex.take();
        pose.x = x;
        pose_mutex.give();
    };
    inline void set_y(float y) {
        pose_mutex.take();
        pose.y = y;
        pose_mutex.give();
    };
    inline void set_heading(AngleTemplate<> heading) {
        pose_mutex.take();
        pose.theta = standard_radians(heading);
        pose_mutex.give();
    }
    inline void set_pose(Pose new_pose) { 
        pose_mutex.take();
        pose = new_pose;
        pose_mutex.give();
    };
    inline void set_position(Point new_position) { 
        pose_mutex.take();
        pose.x = new_position.x;
        pose.y = new_position.y;
        pose_mutex.give();
    };
    inline void set(Pose new_pose) {
        pose_mutex.take();
        pose = new_pose;
        pose_mutex.give();
        pf->set_particles_point(Point(new_pose.x, new_pose.y));
    };
    inline void set(Point new_position) {
        pose_mutex.take();
        pose.x = new_position.x;
        pose.y = new_position.y;
        pose_mutex.give();
        pf->set_particles_point(new_position);
    };
    
    void distance_reset(Pose estimate, float particle_spread = 0.0f);
    Pose compute_odometry_delta();
    void update_position();
    void calibrate();

    // void update_accel();
    // float get_accel_x();
    // float get_vel_x();
    // float get_pos_x();

private:
    std::shared_ptr<MotorGroup> left_motors;
    std::shared_ptr<MotorGroup> right_motors;
    std::shared_ptr<pros::Imu> imu;
    std::shared_ptr<ParticleFilter> pf;
    Pose pose;
    bool use_particle_filtering = true;
};

} // namespace miku