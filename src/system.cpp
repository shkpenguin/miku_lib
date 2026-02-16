#include "system.hpp"
#include "config.hpp"
#include "miku/miku-api.hpp"
#include "fmt/core.h"
#include "macro.hpp"
#include <deque>
#include <vector>

#define LUT_TESTING 0

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
    display_dist_sensors,
    display_odom_raw,
    display_motor_temps,
    display_drive_vel,
    display_intake_vel,
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

int lut_test_voltage = 0;

void teleop_intake_control() {
    
    while(true) {

        #if LUT_TESTING 
        if(master.get_digital(DIGITAL_B)) {
            intake.set(lut_test_voltage, lut_test_voltage);
        }

        else {
        #else
        if(master.get_digital_new_press(DIGITAL_LEFT)) intake.queue_command(-12000, -12000, -12000, 100);
        else if(master.get_digital(DIGITAL_LEFT)) {
            
            intake.set_top_velocity(-100);
            // intake.set_middle_velocity(200);
            // intake.set_top(-4000);
            intake.set_middle_velocity(100);
            intake.set_bottom(6000);
        }

        if(master.get_digital(DIGITAL_RIGHT)) {
            intake.set_top(12000);
            intake.set_middle(-12000);
            intake.set_bottom(0);
        }
        
        #endif

        if(master.get_digital(DIGITAL_A)) middle_piston.toggle();

        bool shift = master.get_digital(DIGITAL_L1);

        if(shift) {

            if(master.get_digital_new_press(DIGITAL_R1)) loader_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                intake.set(-12000, -12000);
            } else if(master.get_digital(DIGITAL_L2)) {
                intake.set(-12000, 12000);
            }

        } else {

            if(master.get_digital_new_press(DIGITAL_R1)) {
                lock_piston.toggle();
                master.set_rumble(!lock_piston.get_value());
            }
            
            if(master.get_digital_new_press(DIGITAL_L2)) descore_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                // if(lock_piston.get_value() == true) intake.set(12000, 12000);
                // else intake.set(12000, 12000);
                intake.set(12000, 12000);
            }

        } // shift

        if(!master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_LEFT) && !master.get_digital(DIGITAL_RIGHT)) {
            intake.stop();
        }

        #if LUT_TESTING 
        }
        #endif

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
                // master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        pros::Task::delay_until(&curr_ms, DELTA_TIME);
    }

}

pros::Task* teleop_intake_task = nullptr;

void teleop_system_control() {

    #if LUT_TESTING
        master.display(0, []() {
            return fmt::format("Voltage: {}mV", lut_test_voltage);
        });
        master.display(1, []() { // drive left/right velocity
            return fmt::format("drive: {} {}", (int) left_motors.get_average_velocity(), (int) right_motors.get_average_velocity());
        });
        master.display(2, []() { // intake top/middle/bottom velocity
            return fmt::format("{} {} {}", (int) intake_top.get_filtered_velocity(), (int) intake_middle.get_filtered_velocity(), (int) intake_bottom.get_filtered_velocity());
        });
    #endif

    teleop_intake_task = new pros::Task([]() {
        teleop_intake_control();
    });

    while (true) {

        uint32_t prev_time = pros::millis();

        master.update_display();
        intake.update();
        Miku.update_position();

        // if(master.get_digital_new_press(DIGITAL_Y)) {
        //     teleop_intake_task->remove();
        //     teleop_intake_task = nullptr;
        //     skills_mid_control();
        // }

        if(master.get_digital_new_press(DIGITAL_Y)) Miku.distance_reset(Miku.get_position());

        #if LUT_TESTING
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
        #endif

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

            if(master.get_digital_new_press(DIGITAL_X)) {
                displayModes.cycle_forward()();
            }

            #if LUT_TESTING
            if(master.get_digital(DIGITAL_B)) {
                Miku.move_voltage(lut_test_voltage, lut_test_voltage);
            }

            else { // not running LUT test
            #else 
            if(master.get_digital_new_press(DIGITAL_B)) {
                drive_modes.cycle_forward();
                master.rumble("-");
            }
            #endif

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

            #if LUT_TESTING
            }
            #endif
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