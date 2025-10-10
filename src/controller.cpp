#define FMT_HEADER_ONLY
#include "fmt/core.h"
#include "miku-api.h"
#include <vector>

Timer rumble_timer;

struct DisplayItem {
    std::function<std::string()> message_func;
    int timeout_ms; // how long to display
    NotificationType m_type = NotificationType::DISPLAY; // default type is DISPLAY
    DisplayItem(std::function<std::string()> func, int timeout, NotificationType type = NotificationType::DISPLAY)
        : message_func(func), timeout_ms(timeout), m_type(type) {}
    DisplayItem() = default; // Default constructor for empty item
};

std::vector<std::pair<std::string, miku::Motor*>> dt_motors = {
    {"LF", &left_front},
    {"LM", &left_middle},
    {"LB", &left_back},
    {"RF", &right_front},
    {"RM", &right_middle},
    {"RB", &right_back}
};

std::vector<std::pair<std::string, miku::Motor*>> intake_motors = {
    {"Bottom", &bottom_intake},
    {"Top", &top_intake}
};

std::vector<DisplayItem> notifs;

std::vector<DisplayItem> code_display = {
    DisplayItem(
        []() -> std::string { 
            return fmt::format("{:.2f}", get_pose().x) + " " + fmt::format("{:.2f}", get_pose().y) + " " + fmt::format("{:.2f}", (get_pose({.degrees = true}).theta)); 
        }, 
        2000, 
        NotificationType::DISPLAY
    ),
    DisplayItem(
        []() -> std::string { 
            return fmt::format("{:.2f}", intake.get_average_velocity() / 6); 
        }, 
        2000, 
        NotificationType::DISPLAY
    ),
    DisplayItem(
        []() -> std::string { 
            return fmt::format("{:.2f}", imu.get_rotation());
        }, 
        2000, 
        NotificationType::DISPLAY
    )
};

std::vector<DisplayItem> temp_display = {
    { []() { 
        std::pair<std::string, miku::Motor*> hottest_motor = dt_motors[0];
        for(int i = 1; i < dt_motors.size(); ++i) {
            if (dt_motors[i].second->get_temperature() > hottest_motor.second->get_temperature()) {
                hottest_motor = dt_motors[i];
            }
        }
        return hottest_motor.first + ": " + std::to_string((int) hottest_motor.second->get_temperature()) + "C";
    }, 2000, NotificationType::DISPLAY },
    { []() { 
        std::pair<std::string, miku::Motor*> hottest_motor = intake_motors[0];
        for(int i = 1; i < intake_motors.size(); ++i) {
            if (intake_motors[i].second->get_temperature() > hottest_motor.second->get_temperature()) {
                hottest_motor = intake_motors[i];
            }
        }
        return hottest_motor.first + ": " + std::to_string((int) hottest_motor.second->get_temperature()) + "C";
    }, 2000, NotificationType::DISPLAY }
};

std::vector<DisplayItem> comp_display = {};

std::vector<std::pair<std::vector<DisplayItem>, std::string>> displays = { {code_display, "Code Debug"}, {temp_display, "Drive Practice"} };

void display_controller() {

    rumble_timer.set(1000);

    int default_index = 0;
    int display_index = 0;

    rumble_timer.pause();

    master.set_text(0, 0, "              ");

    DisplayItem current = displays[display_index].first[default_index];

    while(true) {

        pros::delay(100);
        if(!get_lock() && rumble_timer.isDone()) {
            master.rumble("--");
            rumble_timer.reset();
            continue;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            master.rumble("-");
            display_index = (display_index + 1) % displays.size();
            default_index = 0;
            master.set_text(0, 0, "              ");
            pros::delay(20);
            current = displays[display_index].first[default_index];
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            master.rumble("-");
            display_index = (display_index - 1 + displays.size()) % displays.size();
            default_index = 0;
            master.set_text(0, 0, "              ");
            pros::delay(20);
            current = displays[display_index].first[default_index];
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            default_index = (default_index - 1 + displays[display_index].first.size()) % displays[display_index].first.size();
            current = displays[display_index].first[default_index];
            master.set_text(0, 0, "              ");
            pros::delay(20);
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            default_index = (default_index + 1) % displays[display_index].first.size();
            current = displays[display_index].first[default_index];
            master.set_text(0, 0, "              ");
            pros::delay(20);
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