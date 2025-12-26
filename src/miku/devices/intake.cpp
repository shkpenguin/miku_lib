#include "miku/devices/intake.hpp"
#include "config.hpp"

namespace miku {

Intake::Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> bottom)
    : top_motor(top), bottom_motor(bottom) {}

void Intake::execute_command(std::shared_ptr<Motor>& motor, const MotorCommand& cmd) {
    if (cmd.type == IntakeCommandType::VOLTAGE) {
        motor->move_voltage(cmd.value);
    } else {
        motor->move_velocity(cmd.value);
    }
}

void Intake::move_normal() {
    execute_command(top_motor, top_command);
    execute_command(bottom_motor, bottom_command);
}

bool Intake::is_jammed() const {
    bool low_velocity = bottom_motor->get_filtered_velocity() < LOW_VELOCITY_THRESHOLD;
    bool commanded_forward = (top_command.value > 0) || (bottom_command.value > 0);
    return low_velocity && commanded_forward;
}

// During an unjam sequence we reverse both rollers at UNJAM_VOLTAGE for
// UNJAM_DURATION_MS to try to clear a stuck object. This is intentionally
// short but aggressive; if the unjam routine is causing problems, try one
// or more of these options:
//  - increase JAM_THRESHOLD_MS so we only unjam on more persistent jams
//  - increase LOW_VELOCITY_THRESHOLD so we require a clearer slowdown
//  - reduce the magnitude of UNJAM_VOLTAGE to be less aggressive
void Intake::handle_unjam(uint32_t now) {
    if (now - unjam_start_time < UNJAM_DURATION_MS) {
        top_motor->move_voltage(UNJAM_VOLTAGE);
        bottom_motor->move_voltage(UNJAM_VOLTAGE);
    } else {
        is_unjamming = false;
        jam_detect_time = 0;
        move_normal();
    }
}

// Monitor the jam condition and require the low velocity state to persist
// for JAM_THRESHOLD_MS before starting an unjam. This avoids reacting to
// brief transients (e.g., a brief load or a momentary sensor glitch).
// If you observe frequent false positives, increase JAM_THRESHOLD_MS or add
// a minimum commanded magnitude check to is_jammed().
void Intake::handle_jam_detection(uint32_t now) {
    if (is_jammed()) {
        if (jam_detect_time == 0) {
            jam_detect_time = now;
        }

        if (now - jam_detect_time > JAM_THRESHOLD_MS) {
            is_unjamming = true;
            unjam_start_time = now;
        } else {
            move_normal();
        }
    } else {
        jam_detect_time = 0;
        move_normal();
    }
}

void Intake::set(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd) {
    top_command = top_cmd;
    bottom_command = bottom_cmd;
}

void Intake::set(float top_voltage, float bottom_voltage) {
    top_command = MotorCommand(top_voltage, IntakeCommandType::VOLTAGE);
    bottom_command = MotorCommand(bottom_voltage, IntakeCommandType::VOLTAGE);
}

void Intake::set_velocity(float top_vel, float bottom_vel) {
    top_command = MotorCommand(top_vel, IntakeCommandType::VELOCITY);
    bottom_command = MotorCommand(bottom_vel, IntakeCommandType::VELOCITY);
}

void Intake::set_top(const MotorCommand& cmd) {
    top_command = cmd;
}

void Intake::set_top(float voltage) {
    top_command = MotorCommand(voltage, mV);
}

void Intake::set_top_velocity(float velocity) {
    top_command = MotorCommand(velocity, rpm);
}

void Intake::set_bottom(const MotorCommand& cmd) {
    bottom_command = cmd;
}

void Intake::set_bottom(float voltage) {
    bottom_command = MotorCommand(voltage, mV);
}

void Intake::set_bottom_velocity(float velocity) {
    bottom_command = MotorCommand(velocity, rpm);
}

void Intake::load() {
    lock_piston.set_value(false);
    anti_jam_enabled = false;
    set(4000.0f, 12000.0f);
}

void Intake::score() {
    lock_piston.set_value(true);
    anti_jam_enabled = true;
    set(12000.0f, 12000.0f);
}

void Intake::stop() {
    anti_jam_enabled = false;
    set(0.0f, 0.0f);
}

void Intake::set_anti_jam(bool enabled) {
    anti_jam_enabled = enabled;
    if (!enabled) {
        is_unjamming = false;
        jam_detect_time = 0;
    }
}

void Intake::update() {
    if (!anti_jam_enabled) {
        move_normal();
        return;
    }

    uint32_t now = pros::millis();

    if (is_unjamming) {
        handle_unjam(now);
    } else {
        handle_jam_detection(now);
    }
}

} // namespace miku
