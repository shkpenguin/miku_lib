#include "system.hpp"
#include "miku/miku-api.hpp"
#include "fmt/core.h"
#include "macro.hpp"
#include <deque>
#include <vector>

pros::Task* autonomous_system_task = nullptr; // if needed by other translation units
std::deque<MotionPrimitive*> motion_queue;
pros::Mutex queue_mutex;
MotionPrimitive* current_motion = nullptr;

uint32_t curr_dt = 0;
uint32_t prev_ms = 0;

List<DriveMode> drive_modes = {
    DriveMode::TANK,
    DriveMode::ARCADE,
    DriveMode::SINGLE_STICK_ARCADE,
    // DriveMode::FUNNY_TANK
};

List<std::function<void()>> displayModes = {
    display_pose,
    display_motor_temps,
    display_vel,
    display_floor_color,
    display_drive_voltage,
};

void queue_motion(MotionPrimitive* motion) {
    queue_mutex.take();
    motion_queue.push_back(motion);
    queue_mutex.give();
}

void queue_after_current(MotionPrimitive* motion) {
    queue_mutex.take();
    if (current_motion == nullptr) {
        motion_queue.push_back(motion);
    } else {
        motion_queue.insert(motion_queue.begin(), motion);
    }
    queue_mutex.give();
}

void teleop_intake_control() {
    
    while(true) {

        bool shift = master.get_digital(DIGITAL_L1);

        if(shift) {

            if(master.get_digital_new_press(DIGITAL_R1)) loader_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                // spit (shift + R2)
                intake.set_top(-6000);
                intake.set_bottom_velocity(-300);
            } else if(master.get_digital_new_press(DIGITAL_L2)) {
                // quick out (shift + L2 press)
                intake.set(-12000, -6000);
                intake.update();
                pros::delay(200);
                continue;
            } else if(master.get_digital(DIGITAL_L2)) {
                // fast intake (shift + L2 hold)
                intake.set_top_velocity(300);
                intake.set_bottom(12000);
            } else {
                // no intake buttons while shifted -> stop
                intake.stop();
            }

        } else {

            if(master.get_digital_new_press(DIGITAL_R1)) {
                lock_piston.toggle();
                master.set_rumble(lock_piston.get_value());
            }
            
            if(master.get_digital_new_press(DIGITAL_L2)) descore_piston.toggle();
            
            if(master.get_digital_new_press(DIGITAL_A)) middle_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                // Main intake in
                if(lock_piston.get_value() == false) intake.set(4000, 12000);
                else intake.set(12000, 12000);
            } else if(master.get_digital_new_press(DIGITAL_LEFT)) {
                // Quick out
                intake.set(-12000, -6000);
                intake.update();
                pros::delay(200);
                continue;
            } else if(master.get_digital(DIGITAL_LEFT)) {
                // Hold left: top velocity 150, bottom depends on optical
                intake.set_top_velocity(150);
                if(intake_optical.get_color(RED)) intake.set_bottom(-12000);
                else intake.set_bottom(8000);
            } else {
                // no intake buttons -> stop
                intake.stop();
            }

        } // shift

        intake.update();
        pros::delay(10);

    }

}

void autonomous_system_control() {

    prev_ms = pros::millis();

    while (true) {

        uint32_t curr_ms = pros::millis();
        curr_dt = curr_ms - prev_ms;
        prev_ms = curr_ms;

        master.update_display();
        Miku.update_position();
        intake.update();

        // If no motion is running, start the next one
        if (current_motion == nullptr) {
            queue_mutex.take();
            if (!motion_queue.empty()) {
                current_motion = motion_queue.front();
                motion_queue.pop_front();
                queue_mutex.give();

                current_motion->start();
            }
            else {
                queue_mutex.give();
            }
        }

        // Update running motion
        if (current_motion) {
            // Trigger motion events first (so events like stop_when() take effect immediately)
            for (auto& e : current_motion->conditional_events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if(!current_motion->sequential_events.empty()) {
                ConditionalEvent& e = current_motion->sequential_events.front();
                if (e.condition()) {
                    e.action();
                    current_motion->sequential_events.pop();
                }
            }

            if (current_motion->is_done()) {
                current_motion = nullptr;
                master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        pros::Task::delay_until(&curr_ms, DELTA_TIME);
    }

}

int lut_test_voltage = 0;
bool lut_testing = false;

pros::Task* teleop_intake_task = nullptr;

void teleop_system_control() {

    if(lut_testing) {
        master.display(0, []() {
            return fmt::format("Voltage: {}mV", lut_test_voltage);
        });
        master.display(1, []() { // drive left/right velocity
            return fmt::format("RPM: {} {}", (int) left_motors.get_average_velocity(), (int) right_motors.get_average_velocity());
        });
    }

    while (true) {

        uint32_t prev_time = pros::millis();

        master.update_display();
        intake.update();
        Miku.update_position();

        if(master.get_digital_new_press(DIGITAL_Y)) skills_mid_control();

        if(lut_testing) {
            if(master.get_digital_new_press(DIGITAL_LEFT)) {
                lut_test_voltage -= 500;
            }
            if(master.get_digital_new_press(DIGITAL_RIGHT)) {
                lut_test_voltage += 500;
            }
            if(master.get_digital_new_press(DIGITAL_UP)) {
                lut_test_voltage += 1000;
            }
            if(master.get_digital_new_press(DIGITAL_DOWN)) {
                lut_test_voltage *= -1;
            }
        }

        bool joysticks_active = 
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10);

        if(!motion_queue.empty() || current_motion != nullptr) {
            if(joysticks_active) {
                // Clear motion queue and stop current motion
                queue_mutex.take();
                motion_queue.clear();
                queue_mutex.give();
                current_motion = nullptr;
                master.rumble("-");
            }
        }

        if(joysticks_active && teleop_intake_task == nullptr) {
            teleop_intake_task = new pros::Task([]() {
                teleop_intake_control();
            });
        }

        if(motion_queue.empty() && current_motion == nullptr) {

            if(lut_testing && !master.get_digital_new_press(DIGITAL_B) && master.get_digital(DIGITAL_B)) {
                Miku.move_voltage(lut_test_voltage, lut_test_voltage);
            }

            else if(master.get_digital_new_press(DIGITAL_B)) {
                drive_modes.cycle_forward();
                master.rumble("-");
            }

            if(master.get_digital_new_press(DIGITAL_X)) {
                displayModes.cycle_forward()();
            }

            
            else { // not running LUT test

            if(master.get_digital(DIGITAL_UP)) {
                int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                arcade(50, turn / 3);
            } else if(master.get_digital(DIGITAL_DOWN)) {
                int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                arcade(-50, turn / 3);
            } else {
                if(drive_modes.get_value() == DriveMode::TANK) {
                    int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
                    tank(left, right);
                } else if(drive_modes.get_value() == DriveMode::ARCADE) {
                    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                    arcade(throttle, turn);
                } else if(drive_modes.get_value() == DriveMode::SINGLE_STICK_ARCADE) {
                    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
                    arcade(throttle, turn);
                } else if(drive_modes.get_value() == DriveMode::FUNNY_TANK) {
                    int left_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
                    int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                    int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
                    funny_tank(left_x, left_y, right_x, right_y);
                }
            }

            }
        }

        else {
        
        if (current_motion == nullptr) {
            queue_mutex.take();
            if (!motion_queue.empty()) {
                current_motion = motion_queue.front();
                motion_queue.pop_front();
                queue_mutex.give();

                current_motion->start();
            }
            else {
                queue_mutex.give();
            }
        }

        // Update running motion
        if (current_motion) {
            // Trigger motion events first (so events like stop_when() take effect immediately)
            for (auto& e : current_motion->conditional_events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if(!current_motion->sequential_events.empty()) {
                ConditionalEvent& e = current_motion->sequential_events.front();
                if (e.condition()) {
                    e.action();
                    current_motion->sequential_events.pop();
                }
            }

            if (current_motion->is_done()) {
                current_motion = nullptr;
                master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        }

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

}