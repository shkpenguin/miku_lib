#pragma once

#include <vector>
#include <queue>
#include <initializer_list>
#include <utility>
#include <memory>
#include "miku/mp.h"
#include "config.h"
#include "miku/time.h"
#include "miku/geometry.h"
#include "miku/pid.h"
#include "miku/devices/chassis.h"

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

inline ConditionalEvent await(std::function<bool()> condition) {
    return {
        condition,
        []() {}
    };
}

inline ConditionalEvent within(const Point& target, float distance, std::function<void()> action) {
    return {
        [target, distance]() { return Miku.get_position().distance_to(target) < distance; },
        action
    };
}

inline ConditionalEvent within(float target, float tolerance, std::function<void()> action) {
    return {
        [target, tolerance]() { 
            compass_degrees current = compass_degrees(Miku.get_heading()).wrap();
            compass_degrees diff = (compass_degrees(target) - current).wrap();
            return diff < tolerance;
        },
        action
    };
}

inline ConditionalEvent elapsed(int ms, std::function<void()> action) {
    // start_time is initialized lazily so the elapsed timer begins when the condition
    // is first checked (useful when used as a sequential event)
    auto start_time = std::make_shared<int>(-1);
    return {
        [start_time, ms]() {
            if (*start_time == -1) *start_time = pros::millis();
            return (pros::millis() - *start_time) >= ms;
        },
        action
    };
}

inline ConditionalEvent start(std::function<void()> action) {
    return {
        []() { return true; },
        action
    };
}

struct MotionPrimitive {
    uint32_t start_time;
    bool done = false;

    virtual void start() = 0;
    virtual void update() = 0;
    virtual void stop() = 0;
    virtual bool is_done() = 0;
    virtual ~MotionPrimitive() {}

    std::vector<ConditionalEvent> conditional_events;
    std::queue<ConditionalEvent> sequential_events;
    void add_conditional_event(const ConditionalEvent& event) {
        conditional_events.push_back(event);
    }
    void add_sequential_event(const ConditionalEvent& event) {
        sequential_events.push(event);
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

    void stop() override {
        done = true;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
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
    void stop() override;
    bool is_done() override;
};

void queue_motion(MotionPrimitive* motion);
void queue_after_current(MotionPrimitive* motion);

struct MotionBuilder {
protected:
    MotionPrimitive* motion;

    explicit MotionBuilder(MotionPrimitive* motion) : motion(motion) {}

public:
    MotionBuilder(const MotionBuilder&) = delete;
    MotionBuilder& operator=(const MotionBuilder&) = delete;
    MotionBuilder(MotionBuilder&& other) noexcept : motion(other.motion) { other.motion = nullptr; }
    MotionBuilder& operator=(MotionBuilder&& other) noexcept {
        if (this != &other) {
            motion = other.motion;
            other.motion = nullptr;
        }
        return *this;
    }
    virtual ~MotionBuilder() = default;

    MotionBuilder& event(const ConditionalEvent& event) {
        motion->add_conditional_event(event);
        return *this;
    }

    MotionBuilder& events(std::initializer_list<ConditionalEvent> events) {
        for (const auto& e : events) {
            motion->add_conditional_event(e);
        }
        return *this;
    }

    MotionBuilder& seq(const ConditionalEvent& event) {
        motion->add_sequential_event(event);
        return *this;
    }

    MotionBuilder& seq(std::initializer_list<ConditionalEvent> events) {
        for (const auto& e : events) {
            motion->add_sequential_event(e);
        }
        return *this;
    }

    MotionPrimitive* queue() {
        queue_motion(motion);
        return motion;
    }

    MotionPrimitive* run() {
        queue_after_current(motion);
        return motion;
    }

    MotionBuilder& end(std::function<bool()> condition) {
        MotionPrimitive* mptr = motion;
        motion->add_conditional_event({
            condition,
            [mptr]() { mptr->stop(); }
        });
        return *this;
    }

    MotionBuilder& end_seq(std::function<bool()> condition) {
        MotionPrimitive* mptr = motion;
        motion->add_sequential_event({
            condition,
            [mptr]() { mptr->stop(); }
        });
        return *this;
    }

    MotionPrimitive* ptr() {
        return motion;
    }
};

struct TurnHeadingBuilder : MotionBuilder {
    TurnHeadingBuilder(compass_degrees target, float timeout, TurnParams params = TurnParams())
        : MotionBuilder(new TurnHeading(target, timeout, params)) {}
};

struct TurnPointBuilder : MotionBuilder {
    TurnPointBuilder(Point target, float timeout, TurnParams params = TurnParams())
        : MotionBuilder(new TurnPoint(target, timeout, params)) {}
};

struct SwingHeadingBuilder : MotionBuilder {
    SwingHeadingBuilder(compass_degrees target, float timeout, SwingParams params = SwingParams())
        : MotionBuilder(new SwingHeading(target, timeout, params)) {}
};

struct SwingPointBuilder : MotionBuilder {
    SwingPointBuilder(Point target, float timeout, SwingParams params = SwingParams())
        : MotionBuilder(new SwingPoint(target, timeout, params)) {}
};

struct MovePointBuilder : MotionBuilder {
    MovePointBuilder(Point target, float timeout, MovePointParams params = MovePointParams())
        : MotionBuilder(new MovePoint(target, timeout, params)) {}
};

struct MovePoseBuilder : MotionBuilder {
    MovePoseBuilder(Point target, compass_degrees heading, float timeout, MovePoseParams params = MovePoseParams())
        : MotionBuilder(new MovePose(target, heading, timeout, params)) {}
};

struct MoveTimeBuilder : MotionBuilder {
    MoveTimeBuilder(float left_speed, float right_speed, float duration)
        : MotionBuilder(new MoveTime(left_speed, right_speed, duration)) {}
};

struct RamseteBuilder : MotionBuilder {
    RamseteBuilder(std::vector<Waypoint> waypoints, float timeout, RamseteParams params = RamseteParams())
        : MotionBuilder(new Ramsete(std::move(waypoints), timeout, params)) {}

    RamseteBuilder(BezierPath& path, float timeout, RamseteParams params = RamseteParams())
        : MotionBuilder(new Ramsete(path.get_waypoints(), timeout, params)) {}
};

struct DelayBuilder : MotionBuilder {
    explicit DelayBuilder(float duration) : MotionBuilder(new Delay(duration)) {}
};

using wait = DelayBuilder;
using turn_heading = TurnHeadingBuilder;
using turn_point = TurnPointBuilder;
using swing_heading = SwingHeadingBuilder;
using swing_point = SwingPointBuilder;
using move_point = MovePointBuilder;
using move_pose = MovePoseBuilder;
using move_time = MoveTimeBuilder;
using ramsete = RamseteBuilder;