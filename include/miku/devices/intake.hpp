#pragma once

#include "miku/devices/motor.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <deque>

enum class IntakeCommandType {
    VOLTAGE,
    VELOCITY
};

constexpr IntakeCommandType mV  = IntakeCommandType::VOLTAGE;
constexpr IntakeCommandType rpm = IntakeCommandType::VELOCITY;

struct MotorCommand {
    float value;
    IntakeCommandType type;
    
    MotorCommand(float val = 0.0f, IntakeCommandType t = IntakeCommandType::VOLTAGE)
        : value(val), type(t) {}
};

namespace miku {

class Intake {
private:
    std::shared_ptr<Motor> top_motor;
    std::shared_ptr<Motor> middle_motor;
    std::shared_ptr<Motor> bottom_motor;

    MotorCommand top_command;
    MotorCommand middle_command;
    MotorCommand bottom_command;

    bool anti_jam_enabled = false;

    uint32_t jam_detect_time = 0;
    uint32_t last_unjam_end_time = 0;

    // Intake command queue: queued commands have priority over the default
    // `*_*_command` values. A queued entry may have a duration (ms); duration
    // == 0 means "run until popped/cleared". The queue is protected by a
    // mutex because commands may be pushed from other tasks (teleop, events).
    struct QueueEntry {
        MotorCommand top;
        MotorCommand middle;
        MotorCommand bottom;
        uint32_t duration_ms = 0;     // 0 => run until removed
        uint32_t start_time = 0;      // will be set when entry starts executing
        bool is_unjam = false;       // convenience flag for anti-jam entries
    };

    std::deque<QueueEntry> command_queue;
    pros::Mutex queue_mutex; // protects command_queue

    // Internal helper used by anti-jam to push the standard unjam entry
    void push_unjam_to_queue(bool front = true);

    // Anti-jam tuning constants:
    // - LOW_VELOCITY_THRESHOLD: bottom motor filtered velocity below this
    //   value (unit: RPM) while commanding intake forward is considered a
    //   possible jam. If you get false positives, try raising this value.
    // - JAM_THRESHOLD_MS: duration (ms) the low-velocity condition must persist
    //   before starting the unjam routine (helps avoid transients).
    // - UNJAM_DURATION_MS: how long (ms) to run the reverse "unjam" action.
    // - UNJAM_VOLTAGE: the voltage applied during the unjam routine (negative
    //   reverses the rollers).
    // Tuning tips: if unjam triggers too often, increase JAM_THRESHOLD_MS,
    // increase LOW_VELOCITY_THRESHOLD, or require a larger commanded-forward
    // magnitude (add a minimum-command threshold in is_jammed()).
    static constexpr float LOW_VELOCITY_THRESHOLD = 2.0f; // rpm
    static constexpr int JAM_THRESHOLD_MS = 500;         // ms
    static constexpr int UNJAM_DURATION_MS = 100;        // ms
    static constexpr int UNJAM_VOLTAGE = -12000;         // mV
    static constexpr uint32_t UNJAM_COOLDOWN_MS = 2000;

    void execute_command(std::shared_ptr<Motor>& motor, const MotorCommand& cmd);

    void move_normal();

    // Returns true if the intake is considered jammed.
    // Criteria: bottom motor filtered velocity < LOW_VELOCITY_THRESHOLD AND
    // either top or bottom command is positive (we're trying to intake).
    // Notes: this simple check can false-trigger when commanded outputs are
    // intentionally small or when the mechanism is loaded but functioning. If
    // you see frequent false positives consider adding a minimum commanded
    // voltage/velocity requirement here or increasing the threshold/timer.
    bool is_jammed() const;

    // (handle_unjam removed — anti-jam now queues an unjam entry.)

    // Monitor the jam condition and only start an unjam sequence if the
    // condition persists for JAM_THRESHOLD_MS to avoid reacting to short
    // transients or sensor noise.
    void handle_jam_detection(uint32_t now);

public:
    Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> middle, std::shared_ptr<Motor> bottom);

    // Three-arg set methods (preferred when using a middle motor)
    void set(const MotorCommand& top_cmd, const MotorCommand& middle_cmd, const MotorCommand& bottom_cmd);
    void set(float top_voltage, float middle_voltage, float bottom_voltage);
    void set_velocity(float top_vel, float middle_vel, float bottom_vel);

    // Backwards-compatible two-arg set methods
    void set(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd);
    void set(float top_voltage, float bottom_voltage);
    void set_velocity(float top_vel, float bottom_vel);

    void set_top(const MotorCommand& cmd);

    void set_top(float voltage);

    void set_top_velocity(float velocity);
    
    void set_middle(const MotorCommand& cmd);
    void set_middle(float voltage);
    void set_middle_velocity(float velocity);

    void set_bottom(const MotorCommand& cmd);

    void set_bottom(float voltage);

    void set_bottom_velocity(float velocity);

    void load();

    void score();

    void stop();

    void set_anti_jam(bool enabled);

    // Queue API — queued entries override the default commands until they
    // complete or are cleared. `front = true` inserts the entry at the
    // head of the queue (higher priority).
    void queue_command(const MotorCommand& top_cmd,
                       const MotorCommand& middle_cmd,
                       const MotorCommand& bottom_cmd,
                       uint32_t duration_ms = 0,
                       bool front = false);

    // Convenience: spin all rollers at `voltage` for `duration_ms` (ms)
    void queue_spin(float voltage, uint32_t duration_ms, bool front = false);

    // Clear queued commands (does not change the default commands).
    void clear_queue();

    // Cancel the active queued command (if any) and start the next queued
    // command (or resume defaults).
    void cancel_current_command();

    // Number of queued entries (not including default command)
    size_t queued_size() const;

    void update();
};

} // namespace miku