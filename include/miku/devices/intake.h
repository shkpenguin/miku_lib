#include "miku/devices/motor.h"
#include "pros/rtos.hpp"

enum IntakeCommandType {
    VOLTAGE,
    VELOCITY
};

struct MotorCommand {
    float number;
    IntakeCommandType type;
};

namespace miku {

class Intake {
private:
    std::shared_ptr<Motor> top_motor;
    std::shared_ptr<Motor> bottom_motor;

    bool anti_jam_enabled = false;

    const int jam_threshold_ms = 500;
    const int unjam_duration_ms = 100;

    uint32_t jam_detect_time = 0;
    uint32_t unjam_start_time = 0;

    bool is_unjamming = false;

    MotorCommand top_command = {0, IntakeCommandType::VOLTAGE};
    MotorCommand bottom_command = {0, IntakeCommandType::VOLTAGE};

public:
    Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> bottom)
        : top_motor(top),
          bottom_motor(bottom) {}

    void set(MotorCommand top_cmd, MotorCommand bottom_cmd) {
        top_command = top_cmd;
        bottom_command = bottom_cmd;
    }

    void set(float top_voltage, float bottom_voltage) {
        top_command = {top_voltage, IntakeCommandType::VOLTAGE};
        bottom_command = {bottom_voltage, IntakeCommandType::VOLTAGE};
    }

    void set_top(MotorCommand cmd) { top_command = cmd; }
    void set_bottom(MotorCommand cmd) { bottom_command = cmd; }

    void stop() {
        set({0, IntakeCommandType::VOLTAGE},
            {0, IntakeCommandType::VOLTAGE});
    }

    void set_anti_jam(bool enabled) {
        anti_jam_enabled = enabled;
    }

    void move_normal() {
        if(top_command.type == IntakeCommandType::VOLTAGE)
            top_motor->move_voltage(top_command.number);
        else
            top_motor->move_velocity(top_command.number);

        if(bottom_command.type == IntakeCommandType::VOLTAGE)
            bottom_motor->move_voltage(bottom_command.number);
        else
            bottom_motor->move_velocity(bottom_command.number);
    }

    void update() {
        if(!anti_jam_enabled) {
            move_normal();
            return;
        }

        uint32_t now = pros::millis();

        // If we are currently unjamming
        if(is_unjamming) {
            if(now - unjam_start_time < unjam_duration_ms) {
                // Reverse motors during unjam
                top_motor->move_voltage(-12000);
                bottom_motor->move_voltage(-12000);
                return;
            } else {
                // End unjam
                is_unjamming = false;
                jam_detect_time = 0;
                move_normal();
                return;
            }
        }

        // Check velocities
        bool low_velocity =
            // top_motor->get_filtered_velocity() < 10.0 ||
            bottom_motor->get_filtered_velocity() < 5.0;

        bool commanded_forward =
            (top_command.number > 0) || (bottom_command.number > 0);

        if(low_velocity && commanded_forward) {
            // Start jam timer
            if(jam_detect_time == 0)
                jam_detect_time = now;

            // If jam lasted long enough, start unjamming
            if(now - jam_detect_time > jam_threshold_ms) {
                is_unjamming = true;
                unjam_start_time = now;
                return;
            }
        } else {
            // Reset if velocities are okay
            jam_detect_time = 0;
        }

        // Otherwise normal movement
        move_normal();
    }
};

} // namespace miku