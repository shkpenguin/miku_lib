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

    static constexpr int JAM_THRESHOLD_MS = 500;
    static constexpr int UNJAM_DURATION_MS = 100;
    static constexpr float LOW_VELOCITY_THRESHOLD = 5.0f;
    static constexpr int UNJAM_VOLTAGE = -12000;

    void execute_command(std::shared_ptr<Motor>& motor, const MotorCommand& cmd) {
        if (cmd.type == IntakeCommandType::VOLTAGE) {
            motor->move_voltage(cmd.value);
        } else {
            motor->move_velocity(cmd.value);
        }
    }

    void move_normal() {
        execute_command(top_motor, top_command);
        execute_command(bottom_motor, bottom_command);
    }

    bool is_jammed() const {
        bool low_velocity = bottom_motor->get_filtered_velocity() < LOW_VELOCITY_THRESHOLD;
        bool commanded_forward = (top_command.value > 0) || (bottom_command.value > 0);
        return low_velocity && commanded_forward;
    }

    void handle_unjam(uint32_t now) {
        if (now - unjam_start_time < UNJAM_DURATION_MS) {
            top_motor->move_voltage(UNJAM_VOLTAGE);
            bottom_motor->move_voltage(UNJAM_VOLTAGE);
        } else {
            is_unjamming = false;
            jam_detect_time = 0;
            move_normal();
        }
    }

    void handle_jam_detection(uint32_t now) {
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

public:
    Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> bottom)
        : top_motor(top), bottom_motor(bottom) {}

    void set(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd) {
        top_command = top_cmd;
        bottom_command = bottom_cmd;
    }

    void set(float top_voltage, float bottom_voltage) {
        top_command = MotorCommand(top_voltage, IntakeCommandType::VOLTAGE);
        bottom_command = MotorCommand(bottom_voltage, IntakeCommandType::VOLTAGE);
    }

    void set_velocity(float top_vel, float bottom_vel) {
        top_command = MotorCommand(top_vel, IntakeCommandType::VELOCITY);
        bottom_command = MotorCommand(bottom_vel, IntakeCommandType::VELOCITY);
    }

    void set_top(const MotorCommand& cmd) { 
        top_command = cmd; 
    }
    
    void set_top(float voltage) {
        top_command = MotorCommand(voltage, mV);
    }

    void set_top_velocity(float velocity) {
        top_command = MotorCommand(velocity, rpm);
    }
    
    void set_bottom(const MotorCommand& cmd) { 
        bottom_command = cmd; 
    }

    void set_bottom(float voltage) {
        bottom_command = MotorCommand(voltage, mV);
    }

    void set_bottom_velocity(float velocity) {
        bottom_command = MotorCommand(velocity, rpm);
    }

    void load() {
        lock_piston.set_value(false);
        anti_jam_enabled = false;
        set(4000.0f, 12000.0f);
    }

    void score() {
        lock_piston.set_value(true);
        anti_jam_enabled = true;
        set(12000.0f, 12000.0f);
    }

    void stop() {
        lock_piston.set_value(false);
        anti_jam_enabled = false;
        set(0.0f, 0.0f);
    }

    void set_anti_jam(bool enabled) {
        anti_jam_enabled = enabled;
        if (!enabled) {
            is_unjamming = false;
            jam_detect_time = 0;
        }
    }

    void update() {
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
};

} // namespace miku