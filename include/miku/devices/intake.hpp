#pragma once

#include "miku/devices/motor.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace miku {

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

class Intake {
private:
    std::shared_ptr<Motor> top_motor;
    std::shared_ptr<Motor> bottom_motor;

    MotorCommand top_command;
    MotorCommand bottom_command;

    bool anti_jam_enabled = false;
    bool is_unjamming = false;
    
    uint32_t jam_detect_time = 0;
    uint32_t unjam_start_time = 0;
    uint32_t last_unjam_end_time = 0;

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
    static constexpr uint32_t UNJAM_COOLDOWN_MS = 1000;

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

    // While unjamming we apply UNJAM_VOLTAGE to both motors for
    // UNJAM_DURATION_MS, then resume normal commands. Make UNJAM_VOLTAGE less
    // aggressive to reduce stress on mechanism if necessary.
    void handle_unjam(uint32_t now);

    // Monitor the jam condition and only start an unjam sequence if the
    // condition persists for JAM_THRESHOLD_MS to avoid reacting to short
    // transients or sensor noise.
    void handle_jam_detection(uint32_t now);

public:
    Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> bottom);

    void set(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd);

    void set(float top_voltage, float bottom_voltage);

    void set_velocity(float top_vel, float bottom_vel);

    void set_top(const MotorCommand& cmd);

    void set_top(float voltage);

    void set_top_velocity(float velocity);
    
    void set_bottom(const MotorCommand& cmd);

    void set_bottom(float voltage);

    void set_bottom_velocity(float velocity);

    void load();

    void score();

    void stop();

    void set_anti_jam(bool enabled);

    void update();
};

} // namespace miku