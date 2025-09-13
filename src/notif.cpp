#define FMT_HEADER_ONLY
#include "fmt/core.h"
#include "api.h"
#include "notif.h"
#include "config.h"
#include "timer.h"
#include "odom.h"
#include "mcl.h"
#include "misc.h"
#include <vector>

Timer item_timer;
Timer rumble_timer;

struct DisplayItem {
    std::function<std::string()> message_func;
    int timeout_ms; // how long to display
    NotificationType m_type = NotificationType::DISPLAY; // default type is DISPLAY
    DisplayItem(std::function<std::string()> func, int timeout, NotificationType type = NotificationType::DISPLAY)
        : message_func(func), timeout_ms(timeout), m_type(type) {}
    DisplayItem() = default; // Default constructor for empty item
};

std::vector<std::pair<std::string, pros::Motor*>> dt_motors = {
    {"LF", &left_front},
    {"LM", &left_middle},
    {"LB", &left_back},
    {"RF", &right_front},
    {"RM", &right_middle},
    {"RB", &right_back}
};

std::vector<std::pair<std::string, pros::Motor*>> intake_motors = {
    {"Bottom", &bottom_intake},
    {"Top", &top_intake}
};

std::vector<DisplayItem> notifs;

std::vector<DisplayItem> code_display = {
    DisplayItem(
        []() -> std::string { 
            if(imu.is_calibrating()) return "IMU calibrating";
            return fmt::format("{:.2f}", getPose().x) + " " + fmt::format("{:.2f}", getPose().y) + " " + fmt::format("{:.2f}", (getPose().theta * (180.0 / M_PI))); 
        }, 
        2000, 
        NotificationType::DISPLAY
    )
    // DisplayItem(
    //     []() -> std::string { 
    //         if(imu.is_calibrating()) return "IMU calibrating";
    //         return fmt::format("{:.2f}", get_expected_reading(getPose(), back.offset)) + " " + 
    //                fmt::format("{:.2f}", get_expected_reading(getPose(), left.offset)) + " " + 
    //                fmt::format("{:.2f}", get_expected_reading(getPose(), right.offset));
    //     }, 
    //     2000, 
    //     NotificationType::DISPLAY
    // )
};

std::vector<DisplayItem> temp_display = {
    { []() { 
        std::pair<std::string, pros::Motor*> hottest_motor = dt_motors[0];
        for(int i = 1; i < dt_motors.size(); ++i) {
            if (dt_motors[i].second->get_temperature() > hottest_motor.second->get_temperature()) {
                hottest_motor = dt_motors[i];
            }
        }
        return hottest_motor.first + ": " + std::to_string((int) hottest_motor.second->get_temperature()) + "C";
    }, 2000, NotificationType::DISPLAY },
    { []() { 
        std::pair<std::string, pros::Motor*> hottest_motor = intake_motors[0];
        for(int i = 1; i < intake_motors.size(); ++i) {
            if (intake_motors[i].second->get_temperature() > hottest_motor.second->get_temperature()) {
                hottest_motor = intake_motors[i];
            }
        }
        return hottest_motor.first + ": " + std::to_string((int) hottest_motor.second->get_temperature()) + "C";
    }, 2000, NotificationType::DISPLAY }
};

std::vector<DisplayItem> comp_display = {};

std::vector<std::pair<std::vector<DisplayItem>, std::string>> displays = { {temp_display, "Drive Practice"}, {code_display, "Code Debug"} };

/*

void display() {
    int default_index = 0;
    int display_index = 0;
    Timer item_timer(0);

    DisplayItem current;
    bool skip_now = false;
    bool go_back_now = false;

    while (true) {

        pros::delay(10);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            display_index = (display_index + 1) % displays.size();
            default_index = 0; 
            item_timer.set(0);
            add_warning(displays[display_index].second, NotificationType::IMPORTANT);
            continue;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            skip_now = true;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            go_back_now = true;
        }

        // Show next item if timer is up or skip/back requested
        if (item_timer.isDone() || skip_now || go_back_now) {
            std::vector<DisplayItem>& curr_display = displays[display_index].first;

            if (!notifs.empty()) {
                current = notifs.front();
                notifs.erase(notifs.begin());

                // Rumble based on type
                switch (current.m_type) {
                    case NotificationType::FUCKED:    master.rumble("--------"); break;
                    case NotificationType::IMPORTANT: master.rumble("..");       break;
                    case NotificationType::WARNING:   master.rumble(".");        break;
                    default: break;
                }

            } else {
                if (!curr_display.empty()) {
                    if (skip_now || go_back_now) {
                        default_index = (default_index - 1 + curr_display.size()) % curr_display.size();
                    } else {
                        default_index = (default_index + 1) % curr_display.size();
                    }
                    current = curr_display[default_index];
                } else {
                    master.clear();
                    skip_now = false;
                    go_back_now = false;
                    continue;  // No items to show
                }
            }

            // Display the current message
            std::string msg = current.message_func();
            master.clear();
            master.set_text(0, 0, msg.c_str());

            item_timer.set(current.timeout_ms);
            skip_now = false;
            go_back_now = false;

        }

    }
}

*/

void display_controller() {

    item_timer.set(0);
    rumble_timer.set(1000);

    int default_index = 0;
    int display_index = 1;

    rumble_timer.pause();

    DisplayItem current;

    while(true) {

        pros::delay(100);
        if(!get_lock() && rumble_timer.isDone()) {
            master.rumble("--");
            rumble_timer.reset();
            continue;
        }

        if(imu.is_calibrating()) {
            master.set_text(0, 0, "IMU Calibrating");
            continue;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            master.rumble("-");
            display_index = (display_index + 1) % displays.size();
            default_index = 0;
            master.set_text(0, 0, "            ");
            pros::delay(10);
            current = displays[display_index].first[default_index];
            item_timer.set(current.timeout_ms);
        }

        else if(item_timer.isDone()) {
            default_index = (default_index + 1) % displays[display_index].first.size();
            current = displays[display_index].first[default_index];
            master.set_text(0, 0, "            ");
            pros::delay(10);
            item_timer.set(current.timeout_ms);
        }

        std::string msg = current.message_func();
        master.set_text(0, 0, msg.c_str());

    }

}

void add_warning(const std::string& warning, NotificationType type) {
    if(type == NotificationType::FUCKED) {
        notifs.insert(notifs.begin(), DisplayItem([warning]() { return "!!! " + warning; }, 5000, NotificationType::FUCKED));
    }
    else if(type == NotificationType::IMPORTANT) {
        notifs.insert(notifs.begin(), DisplayItem([warning]() { return "!! " + warning; }, 3000, NotificationType::IMPORTANT));
    }
    else if(type == NotificationType::WARNING) {
        notifs.push_back(DisplayItem([warning]() { return "! " + warning; }, 2000, NotificationType::WARNING));
    }
    else if(type == NotificationType::DISPLAY) {
        std::cerr << "Error: Cannot add DISPLAY type notification directly" << std::endl;
    }
}