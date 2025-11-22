#include "main.h"
#include "routes.h"
#include "miku/miku-api.h"
#include "fmt/core.h"
#include <deque>
#include <vector>

pros::Task* autonomous_system_task = nullptr;

std::deque<MotionPrimitive*> motion_queue;
pros::Mutex queue_mutex;
MotionPrimitive* current_motion = nullptr;

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

int selected_idx = 2;
std::vector<Route> routes;

void precalculate_paths() {
    routes.push_back(Route("test route", {24, -48, M_PI_2}, test, test_paths));
    routes.push_back(Route("skills", {14, -48, M_PI}, skills, skills_paths));
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

    optical.set_led_pwm(100);
    optical.set_integration_time(10);

    precalculate_paths();

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

    master.clear();

}

void autonomous() {

    Timer flush_timer(1000);
    file.open("log.txt");

    Miku.reset(routes[selected_idx].start_pose);
    initialize_particles_point(Miku.get_position());

    routes[selected_idx].queue();
    master.display(0, []() {
        return "queuelen: " + std::to_string(motion_queue.size());
    });
    master.display(1, []() {
        return Miku.get_pose().to_string();
    });
    autonomous_system_task = new pros::Task([&flush_timer]() {
    while (true) {
        uint32_t prev_time = pros::millis();

        if(flush_timer.is_done()) {
            flush_logs();
            flush_timer.set(1000);
        }

        master.update_display();
        Miku.update_odometry();

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
            bool done = current_motion->is_done();

            // Trigger motion events
            for (auto& e : current_motion->events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if (done) {
                current_motion = nullptr;
                master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }
    });

}

enum class DriveMode {
    TANK = 0,
    ARCADE = 1,
    FUNNY_TANK = 2
};

int curve(int pos) {
    if(fabs(pos) <= 5) return 0;
    return (pos * pos * pos * 0.00337827447) + (pos * 40);
}

void tank(int left, int right) {
    Miku.move(left, right);
}

void funny_tank(int left_x, int left_y, int right_x, int right_y) {
    if(fabs(left_x) > 50 && fabs(right_x) > 50) {
        int sign = (left_x > 0 || right_x < 0) ? 1 : -1;
        left_x = (fabs(left_x) - 50) * 127 / 77;
        right_x = (fabs(right_x) - 50) * 127 / 77;
        double speed = (fabs(left_x) + fabs(right_x)) / 2.0 * sign;
        Miku.move(speed, speed);
    } else {
        Miku.move(left_y, right_y);
    }
}

void arcade(int throttle, int turn) {
    int left = throttle + turn;
    int right = throttle - turn;

    Miku.move(left, right);
}

List<DriveMode> driveModes = {
    DriveMode::TANK,
    DriveMode::ARCADE,
};

void opcontrol() {

    // /*

    optical.set_led_pwm(0);

    if(autonomous_system_task != nullptr) autonomous_system_task->remove();
    Miku.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // static Gif gif("/usd/jiachenma.gif", lv_scr_act());

    // display motor temps
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

    int drive_vel = 0;
    bool l2_pressed = false;

    while (true) {

        master.update_display();

        if(master.get_digital(DIGITAL_LEFT) && master.get_digital_new_press(DIGITAL_RIGHT)) {
            driveModes.cycle_forward();
            master.rumble("-");
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

        bool shift1 = master.get_digital(DIGITAL_L1);

        if(shift1) {
            if(master.get_digital_new_press(DIGITAL_R1)) loader_piston.toggle();
            if(master.get_digital(DIGITAL_L2)) {
                intake_top.move_voltage(8000);
                intake_bottom.move_voltage(12000);
            }
            if(master.get_digital_new_press(DIGITAL_R2)) {
                intake_top.move_voltage(-12000);
                intake_bottom.move_voltage(-12000);
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
            if(master.get_digital(DIGITAL_R2)) {
                intake_bottom.move_voltage(12000);
                if(!lock_piston.get_value()) intake_top.move_voltage(4000);
                else intake_top.move_voltage(12000);
            }
            if(master.get_digital_new_press(DIGITAL_L2)) descore_piston.toggle();            
        }

        if(!master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_A)
            && !master.get_digital(DIGITAL_UP) && !master.get_digital(DIGITAL_DOWN)) {
            intake_top.move_voltage(0);
            intake_bottom.move_voltage(0);
        }

        pros::delay(10);
    }

    // */

}