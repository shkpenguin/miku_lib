#include "main.h"
#include "routes.h"
#include "miku/miku-api.h"
#include "fmt/core.h"
#include "macro.h"
#include <deque>
#include <vector>

int selected_idx = 3;
std::vector<Route> routes;

void precalculate_paths() {
    routes.push_back(Route("test route", {24, -48, -M_PI_2}, test, test_paths));
    routes.push_back(Route("skills", {14, -46, M_PI_2}, skills, skills_paths));
    routes.push_back(Route("sawp", {0, -48, M_PI}, sawp, sawp_paths));
    routes.push_back(Route("right rush", {14, -46, M_PI_2}, right_rush, rush_paths));
    for(auto& path : routes[selected_idx].paths) {
        path.get().calculate_waypoints();
    }
}

void initialize() {

    left_motors.tare_position();
    right_motors.tare_position();
    intake_top.tare_position();
    intake_bottom.tare_position();

    Miku.set_brake_mode(DEFAULT_AUTONOMOUS_BRAKE_MODE);

    intake_optical.set_led_pwm(100);
    intake_optical.set_integration_time(10);
    floor_optical.set_led_pwm(100);
    floor_optical.set_integration_time(10);

    precalculate_paths();

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

    master.clear();

    Miku.set(routes[selected_idx].start_pose);
    Miku.calibrate();
    routes[selected_idx].queue();

}

uint32_t curr_dt = 0;
uint32_t prev_ms = pros::millis();

inline void display_motor_temps() {
    master.display(0, []() {
        return "top: " + std::to_string(int(intake_top.get_temperature())) + "C";
    });
    master.display(1, []() {
        return "bottom: " + std::to_string(int(intake_bottom.get_temperature())) + "C";
    });
    master.display(2, []() {
        int temp = std::max(left_motors.get_highest_temperature(), right_motors.get_highest_temperature());
        return "drive: " + std::to_string(temp) + "C";
    });
}

inline void display_pose() {
    master.display(0, []() {
        return "queuelen: " + std::to_string(motion_queue.size());
    });
    master.display(1, []() {
        return Miku.get_pose().to_string();
    });
    master.display(2, []() {
        return "dt: " + std::to_string(curr_dt) + "ms";
    });
}

inline void display_vel() {
    master.display(0, []() {
        return fmt::format("top: {:.1f}rpm", intake_top.get_filtered_velocity());
    }); 
    master.display(1, []() {
        return fmt::format("bottom: {:.1f}rpm", intake_bottom.get_filtered_velocity());
    });
    master.display(2, []() {
        return fmt::format("drive: {:.0f} {:.0f}", left_motors.get_average_velocity(), right_motors.get_average_velocity());
    });
}

inline void display_floor_color() {
    master.display(0, []() {
        if(floor_optical.get_color(RED)) return fmt::format("red");
        else if(floor_optical.get_color(BLUE)) return fmt::format("blue");
        else if(floor_optical.get_color(TILE)) return fmt::format("tile");
        else return fmt::format("none");
    });
    master.display(1, []() {
        return fmt::format("hue: {:.1f}", floor_optical.get_hue());
    });
    master.display(2, []() {
        return fmt::format("prox: {:.1f}", (float)floor_optical.get_proximity());
    });
}

void autonomous() {

    display_pose();
    descore_piston.set_value(true);

    autonomous_system_task = new pros::Task([]() {

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
    });

}

enum class DriveMode {
    TANK = 0,
    ARCADE = 1,
    FUNNY_TANK = 2
};

int curve(int pos) {
    if(abs(pos) <= 5) return 0;
    return 10502.7578057 * (std::exp(0.006 * abs(pos)) - 1.0) * (pos > 0 ? 1 : -1);
}

void tank(int left, int right) {
    Miku.move_voltage(curve(left), curve(right));
}

void funny_tank(int left_x, int left_y, int right_x, int right_y) {
    if(abs(left_x) > 50 && abs(right_x) > 50) {
        int sign = (left_x > 0 || right_x < 0) ? 1 : -1;
        left_x = (abs(left_x) - 50) * 127 / 77;
        right_x = (abs(right_x) - 50) * 127 / 77;
        float speed = (abs(left_x) + abs(right_x)) / 2.0 * sign;
        Miku.move(speed, speed);
    } else {
        Miku.move(left_y, right_y);
    }
}

void arcade(int throttle, int turn) {
    int left = curve(throttle + turn);
    int right = curve(throttle - turn);

    Miku.move_voltage(left, right);
}

List<DriveMode> driveModes = {
    DriveMode::TANK,
    DriveMode::ARCADE,
};

List<std::function<void()>> displayModes = {
    display_pose,
    display_motor_temps,
    display_vel,
    display_floor_color,
};

void opcontrol() {

    // /*

    motion_queue.clear();
    current_motion = nullptr;

    if(autonomous_system_task != nullptr) autonomous_system_task->remove();

    Miku.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // static Gif gif("/usd/jiachenma.gif", lv_scr_act());

    pros::Task intake_task([]() {

        while(true) {

            bool shift = master.get_digital(DIGITAL_L1);

            if(shift) {

            if(master.get_digital_new_press(DIGITAL_R1)) loader_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                intake_top.move_voltage(-8000);
                intake_bottom.move_velocity(-300);
            } else if(master.get_digital_new_press(DIGITAL_L2)) {
                intake_top.move_voltage(-12000);
                intake_bottom.move_voltage(-3000);
                pros::delay(300);
            } else if(master.get_digital(DIGITAL_L2)) {
                intake_top.move_voltage(12000);
                intake_bottom.move_voltage(8000);
            }

            } else {

            if(master.get_digital_new_press(DIGITAL_R1)) {
                lock_piston.toggle();
                if(lock_piston.get_value() == true) {
                    master.set_rumble(true);
                } else {
                    master.set_rumble(false);
                }
            }
            
            if(master.get_digital_new_press(DIGITAL_L2)) descore_piston.toggle();
            
            if(master.get_digital_new_press(DIGITAL_A)) middle_piston.toggle();

            if(master.get_digital(DIGITAL_R2)) {
                intake_bottom.move_voltage(12000);
                if(lock_piston.get_value() == false) intake_top.move_voltage(4000);
                else intake_top.move_voltage(12000);
            } else if(master.get_digital_new_press(DIGITAL_LEFT)) {
                intake_top.move_voltage(-12000);
                intake_bottom.move_voltage(-3000);
                pros::delay(300);
            } else if(master.get_digital(DIGITAL_LEFT)) {
                if(intake_optical.get_color(RED)) {
                    intake_bottom.move_voltage(-6000);
                }
                intake_top.move_velocity(150);
                if(intake_bottom.get_commanded_voltage() != -6000) intake_bottom.move_voltage(8000);
            }

            } // shift

            if(!master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_LEFT)) {
                intake_top.move_voltage(0);
                if(intake_bottom.get_filtered_velocity() < 0) {
                    intake_bottom.move_velocity(0);
                } else {
                    intake_bottom.move_voltage(0);
                }
            }

            pros::delay(10);

        }
        
    });

    pros::Task teleop_system_control([]() {

    Miku.set_use_particle_filtering(false);

    while (true) {

        uint32_t prev_time = pros::millis();

        master.update_display();
        Miku.update_position();

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

        if(motion_queue.empty() && current_motion == nullptr) {
            if(master.get_digital(DIGITAL_B) && master.get_digital_new_press(DIGITAL_Y)) {
                driveModes.cycle_forward();
                master.rumble("-");
            }

            if(master.get_digital_new_press(DIGITAL_X)) {
                displayModes.cycle_forward()();
            }

            if(master.get_digital_new_press(DIGITAL_LEFT)) {
                descore_align();
            }

            if(master.get_digital(DIGITAL_UP)) {
                int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                arcade(50, turn / 3);
            } else if(master.get_digital(DIGITAL_DOWN)) {
                int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                arcade(-50, turn / 3);
            } else {
                if(driveModes.get_value() == DriveMode::TANK) {
                    int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
                    tank(left, right);
                } else if(driveModes.get_value() == DriveMode::ARCADE) {
                    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                    arcade(throttle, turn);
                } else if(driveModes.get_value() == DriveMode::FUNNY_TANK) {
                    int left_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
                    int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                    int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                    int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
                    funny_tank(left_x, left_y, right_x, right_y);
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
    });

}