#pragma once

#include "pros/misc.h"
#include "miku/util.hpp"
#include <deque>
#include "miku/motions.hpp"
#include "miku/devices/controller.hpp"
#include "api.h"

extern pros::Task* autonomous_system_task;
extern std::deque<MotionPrimitive*> motion_queue;
extern pros::Mutex queue_mutex;
extern MotionPrimitive* current_motion;

extern uint32_t curr_dt;
extern uint32_t prev_ms;

void queue_motion(MotionPrimitive* motion);
void queue_after_current(MotionPrimitive* motion);

enum class DriveMode {
    TANK = 0,
    ARCADE = 1,
    SINGLE_STICK_ARCADE = 2,
    FUNNY_TANK = 3
};

extern List<DriveMode> drive_modes;

int curve(int pos);

void tank(int left, int right);

void funny_tank(int left_x, int left_y, int right_x, int right_y);

void arcade(int throttle, int turn);

inline void display_drive_voltage() {
    if(drive_modes.get_value() == DriveMode::TANK) {
        master.display(0, []() -> std::string {
            return fmt::format("left: {}mV", curve(master.get_analog(ANALOG_LEFT_Y)));
        });
        master.display(1, []() -> std::string {
            return fmt::format("right: {}mV", curve(master.get_analog(ANALOG_RIGHT_Y)));
        });
    } else if(drive_modes.get_value() == DriveMode::ARCADE) {
        master.display(0, []() -> std::string {
            return fmt::format("fwd: {}mV", curve(master.get_analog(ANALOG_LEFT_Y)));
        });
        master.display(1, []() -> std::string {
            return fmt::format("turn: {}mV", curve(master.get_analog(ANALOG_RIGHT_X)));
        });
    }
}

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

inline void display_odom_raw() {
    master.display(0, []() {
        return fmt::format("left: {:.1f}", left_motors.get_average_position());
    });
    master.display(1, []() {
        return fmt::format("right: {:.1f}", right_motors.get_average_position());
    });
    master.display(2, []() {
        return fmt::format("imu: {:.1f}", imu.get_heading());
    });
}

inline void display_drive_vel() {
    master.display(0, []() {
        return fmt::format("left: {:.1f}rpm", left_motors.get_average_velocity());
    }); 
    master.display(1, []() {
        return fmt::format("right: {:.1f}rpm", right_motors.get_average_velocity());
    });
}

inline void display_intake_vel() {
    master.display(0, []() {
        return fmt::format("top: {:.1f}rpm", intake_top.get_filtered_velocity());
    });
    master.display(1, []() {
        return fmt::format("mid: {:.1f}rpm", intake_middle.get_filtered_velocity());
    });
    master.display(2, []() {
        return fmt::format("bottom: {:.1f}rpm", intake_bottom.get_filtered_velocity());
    });
}
inline void display_dist_sensors() {
    master.display(0, []() {
        return fmt::format("{} {} {} {}", (int)(front_distance.get() / 25.4), (int)(back_distance.get() / 25.4), (int)(left_distance.get() / 25.4), (int)(right_distance.get() / 25.4));
    });
    master.display(1, []() {
        return fmt::format("{:.0f} {:.0f} {:.0f} {:.0f}", get_expected_reading(Miku.get_position(), std::make_shared<miku::Distance>(front_distance), cos(Miku.get_heading()), sin(Miku.get_heading())).distance, 
                                            get_expected_reading(Miku.get_position(), std::make_shared<miku::Distance>(back_distance), cos(Miku.get_heading()), sin(Miku.get_heading())).distance,
                                            get_expected_reading(Miku.get_position(), std::make_shared<miku::Distance>(left_distance), cos(Miku.get_heading()), sin(Miku.get_heading())).distance,
                                            get_expected_reading(Miku.get_position(), std::make_shared<miku::Distance>(right_distance), cos(Miku.get_heading()), sin(Miku.get_heading())).distance);
    });
    master.display(2, []() {
        return fmt::format("pos: {:.1f} {:.1f}", Miku.get_x(), Miku.get_y());
    });
}

inline void display_floor_color() {
    // master.display(0, []() {
    //     if(floor_optical.get_color(RED)) return fmt::format("red");
    //     else if(floor_optical.get_color(BLUE)) return fmt::format("blue");
    //     else if(floor_optical.get_color(TILE)) return fmt::format("tile");
    //     else return fmt::format("none");
    // });
    // master.display(1, []() {
    //     return fmt::format("hue: {:.1f}", floor_optical.get_hue());
    // });
    // master.display(2, []() {
    //     return fmt::format("prox: {:.1f}", (float)floor_optical.get_proximity());
    // });
}

extern List<std::function<void()>> displayModes;

void teleop_system_control();

void autonomous_system_control();

void teleop_intake_control();