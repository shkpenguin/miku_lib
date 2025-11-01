#pragma once

#include <vector>
#include "miku-api.h"

enum MotionType {
    TURN_HEADING,
    TURN_POINT,
    SWING_HEADING,
    SWING_POINT,
    MOVE_POINT,
    MOVE_POSE,
    MOVE_TIME,
    RAMSETE
};

enum class Side {
    LEFT,
    RIGHT,
    AUTO
};

struct ConditionalEvent {
    std::function<bool()> condition;
    std::function<void()> action;
    bool triggered = false;
};

struct MotionPrimitive {
    virtual void start() = 0;
    virtual void update() = 0;
    virtual bool is_done() = 0;
    virtual ~MotionPrimitive() {}

    std::vector<ConditionalEvent> events;
};

void move_motors(double l, double r);

void stop_motors();

void set_drive_brake(pros::motor_brake_mode_e mode);

// Motion control functions
void wait_until_done();
void wait_until_within(Point target, double threshold);
void wait_until_within(double target_angle, double threshold);

void request_motion_start();
void end_motion();
void cancel_motion();
void cancel_all_motions();
bool get_motion_running();

struct TurnParams {
    bool reverse = false;
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 0;
    double kP = -1.0;
    double kI = -1.0;
    double kD = -1.0;
};

struct SwingParams {
    bool reverse = false;
    Side locked_side = Side::AUTO;
    bool hold = true; // hold locked side during swing
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 0;
    double kP = -1.0;
    double kI = -1.0;
    double kD = -1.0;
};

struct MovePointParams {
    bool reverse = false;
    double cutoff = -1.0;
    double max_speed = 12000;
    double min_speed = 1000;
    double drive_kP = -1.0;
    double drive_kI = -1.0;
    double drive_kD = -1.0;
    double turn_kP = -1.0;
    double turn_kI = -1.0;
    double turn_kD = -1.0;
};

struct MovePoseParams {
    bool reverse = false;
    double cutoff = -1.0;
    double max_vel = 100;
    double min_vel = 0;
    double k1 = -1.0;
    double k2 = -1.0;
    double k3 = -1.0;
    double end_cutoff = 6.0;
};

struct RamseteParams {
    bool reverse = false;
    double cutoff = -1.0;
    double max_vel = 100;
    double min_vel = 0;
    double b = -1.0;
    double zeta = -1.0;
    double time_multi = -1.0;
    double end_cutoff = 6.0;
};

struct TurnHeading : MotionPrimitive {
    double target;
    double timeout;
    TurnParams params;

    PID turn_pid;
    Timer timer;

    bool done = false;

    TurnHeading::TurnHeading(double target, double timeout, TurnParams params);

    void start() override;
    void update() override;
    bool is_done() override;
};

struct TurnPoint : MotionPrimitive {
    Point target;
    double timeout;
    TurnParams params;

    PID turn_pid;
    Timer timer;

    bool done = false;

    TurnPoint::TurnPoint(Point target, double timeout, TurnParams params);
  
    void start() override;
    void update() override;
    bool is_done() override;
};

struct SwingHeading : MotionPrimitive {
    double target;
    double timeout;
    SwingParams params;

    PID turn_pid;
    Timer timer;

    bool done = false;

    SwingHeading::SwingHeading(double target, double timeout, SwingParams params);

    void start() override;
    void update() override;
    bool is_done() override;

};

struct SwingPoint : MotionPrimitive {
    Point target;
    double timeout;
    SwingParams params;

    PID turn_pid;
    Timer timer;

    bool done = false;

    SwingPoint::SwingPoint(Point target, double timeout, SwingParams params);

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MovePoint : MotionPrimitive {
    Point target;
    double timeout;
    MovePointParams params;

    PID drive_pid;
    PID turn_pid;
    Timer timer;

    bool done = false;

    MovePoint::MovePoint(Point target, double timeout, MovePointParams params);

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MovePose : MotionPrimitive {
    Pose target;
    double timeout;
    double k1, k2, k3;
    double target_rad;
    MovePoseParams params;

    Timer timer;

    bool done = false;

    MovePose::MovePose(Pose target, double timeout, MovePoseParams params);

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MoveTime : MotionPrimitive {
    double left_speed;
    double right_speed;
    double duration;

    Timer timer;
    bool done = false;

    MoveTime(double left_speed, double right_speed, double duration)
        : left_speed(left_speed), right_speed(right_speed), duration(duration) {}

    void start() override {
        done = false;
        timer.set(duration);
        timer.reset();
        move_motors(left_speed, right_speed);
    }

    void update() override {
        if (timer.is_done()) {
            done = true;
            return;
        }
    }

    bool is_done() override {
        return done;
    }
};

struct Ramsete : MotionPrimitive {
    std::vector<Waypoint> waypoints;
    double timeout;
    RamseteParams params;
    double zeta, b, time_multi;

    Timer timer;
    bool done = false;

    double zeta = 0.7;
    double time_multi = 1.2;
    double b = 0.003;

    bool isFinished = false;

    int waypoint_num = waypoints.size();
    double end_time = waypoints.back().t / time_multi;

    double end_x = waypoints.back().x;
    double end_y = waypoints.back().y;

    int current_waypoint = 0;
    int closest_waypoint = 0;
    double time_passed = 0;
    double time_ahead = 0;

    Ramsete(std::vector<Waypoint> waypoints, double timeout, RamseteParams params);

    void start() override;
    void update() override;
    bool is_done() override;
};