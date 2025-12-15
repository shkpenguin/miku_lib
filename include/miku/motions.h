#pragma once

#include <vector>
#include "miku/mp.h"
#include "config.h"
#include "miku/time.h"
#include "miku/geometry.h"
#include "miku/pid.h"
#include "miku/devices/chassis.h"

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
    uint32_t start_time;
    bool done = false;

    virtual void start() = 0;
    virtual void update() = 0;
    virtual bool is_done() = 0;
    virtual ~MotionPrimitive() {}

    std::vector<ConditionalEvent> events;
    void add_event(const ConditionalEvent& event) {
        events.push_back(event);
    }
};

struct TurnParams {
    bool reverse = false;
    float cutoff = -1.0;
    float max_volt_pct = 100;
    float min_volt_pct = 10;
    float kP = -1.0;
    float kI = -1.0;
    float kD = -1.0;
};

struct SwingParams {
    bool reverse = false;
    Side locked_side = Side::AUTO;
    bool hold = true; // hold locked side during swing
    float cutoff = -1.0;
    float max_volt_pct = 100;
    float min_volt_pct = 10;
    float kP = -1.0;
    float kI = -1.0;
    float kD = -1.0;
};

struct MovePointParams {
    bool reverse = false;
    float cutoff = -1.0;
    float drive_max_volt_pct = 100;
    float turn_max_volt_pct = 50;
    float min_volt_pct = 0;
    float cos_scale = 1.0;
    float drive_kP = -1.0;
    float drive_kI = -1.0;
    float drive_kD = -1.0;
    float turn_kP = -1.0;
    float turn_kI = -1.0;
    float turn_kD = -1.0;
};

struct MovePoseParams {
    bool reverse = false;
    float cutoff = -1.0;
    float max_vel_pct = 50;
    float min_vel_pct = 0;
    float k1 = -1.0;
    float k2 = -1.0;
    float k3 = -1.0;
    float end_cutoff = 6.0;
};

struct RamseteParams {
    bool reverse = false;
    float cutoff = -1.0;
    float b = -1.0;
    float zeta = -1.0;
    float time_multi = -1.0;
    float end_cutoff = 6.0;
};

struct Delay : MotionPrimitive {
    float duration;
    Timer timer;
    

    Delay(float duration) :  duration(duration) {}

    void start() override {
        start_time = pros::millis();
        done = false;
        timer.set(duration);
        timer.reset();
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

struct TurnHeading : MotionPrimitive {
    compass_degrees target;
    float timeout;
    TurnParams params;

    PID turn_pid;
    Timer timer;

    TurnHeading(compass_degrees target, float timeout, TurnParams params = TurnParams());

    void start() override;
    void update() override;
    bool is_done() override;
};

struct TurnPoint : MotionPrimitive {
    Point target;
    float timeout;
    TurnParams params;

    float prev_deg;
    PID turn_pid;
    Timer timer;

    TurnPoint(Point target, float timeout, TurnParams params = TurnParams());
  
    void start() override;
    void update() override;
    bool is_done() override;
};

struct SwingHeading : MotionPrimitive {
    compass_degrees target;
    float timeout;
    SwingParams params;

    PID turn_pid;
    Timer timer;

    SwingHeading(compass_degrees target, float timeout, SwingParams params = SwingParams());
    void start() override;
    void update() override;
    bool is_done() override;

};

struct SwingPoint : MotionPrimitive {
    Point target;
    float timeout;
    SwingParams params;

    PID turn_pid;
    Timer timer;

    SwingPoint(Point target, float timeout, SwingParams params = SwingParams());

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MovePoint : MotionPrimitive {
    Point target;
    float timeout;
    MovePointParams params;

    int start_side;

    PID drive_pid;
    PID turn_pid;
    Timer timer;

    MovePoint(Point target, float timeout, MovePointParams params = MovePointParams());

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MovePose : MotionPrimitive {
    Point target;
    standard_radians target_heading;
    float timeout;
    float k1, k2, k3;
    MovePoseParams params;

    Timer timer;

    MovePose(Point target, compass_degrees heading, float timeout, MovePoseParams params = MovePoseParams());

    void start() override;
    void update() override;
    bool is_done() override;
};

struct MoveTime : MotionPrimitive {
    float left_speed;
    float right_speed;
    float duration;

    Timer timer;

    MoveTime(float left_speed, float right_speed, float duration);

    void start() override;

    void update() override;

    bool is_done() override;

};

struct Ramsete : MotionPrimitive {
    std::vector<Waypoint> waypoints;
    float timeout;
    RamseteParams params;

    Timer timer;
    
    float zeta = 0.7;
    float time_multi = 1.2;
    float b = 0.003;

    bool isFinished = false;

    int waypoint_num = waypoints.size();
    float end_time = waypoints.back().t / time_multi;

    float end_x = waypoints.back().x;
    float end_y = waypoints.back().y;

    int current_waypoint = 0;
    int closest_waypoint = 0;
    float time_passed = 0;
    float time_ahead = 0;

    Ramsete(std::vector<Waypoint> waypoints, float timeout, RamseteParams params = RamseteParams());

    void start() override;
    void update() override;
    bool is_done() override;
};