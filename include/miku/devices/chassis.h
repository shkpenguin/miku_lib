#pragma once

#include "pros/motors.hpp"
#include "miku/devices/motor.h"
#include "miku/geometry.h"
#include <memory>

inline pros::Mutex pose_mutex;

namespace miku {

class Chassis {
public:
    Chassis();
    Chassis(std::shared_ptr<MotorGroup> left, std::shared_ptr<MotorGroup> right)
        : left_motors(left), right_motors(right) {}

    void move(double left_speed, double right_speed) {
        left_motors->move(left_speed);
        right_motors->move(right_speed);
    }
    void move_voltage(double left_speed, double right_speed) {
        left_motors->move_voltage(left_speed);
        right_motors->move_voltage(right_speed);
    }
    void move_velocity(double left_velocity, double right_velocity) {
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
 
    inline Pose get_pose() { return pose; };
    inline Pose get_pose_delta() { return pose_delta; };
    inline Point get_position() { return Point(pose.x, pose.y); };
    inline Point get_position_delta() { return Point(pose_delta.x, pose_delta.y); };
    inline double get_x() { return pose.x; };
    inline double get_y() { return pose.y; };
    inline standard_radians get_heading() { return pose.theta; };
    inline standard_radians get_heading_delta() { return pose_delta.theta; };
    inline void set_x(double x) {
        pose_mutex.take();
        pose.x = x;
        pose_mutex.give();
    };
    inline void set_y(double y) {
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
    void reset(Pose initial_pose); // declared in odom.cpp
    void update_odometry();

private:
    std::shared_ptr<MotorGroup> left_motors;
    std::shared_ptr<MotorGroup> right_motors;
    Pose pose;
    Pose pose_delta;
};

} // namespace miku