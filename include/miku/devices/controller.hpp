#pragma once

#define CONTROLLER_UPDATE_RATE_MS 100

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <functional>
#include <set>

namespace miku {

class Controller : public pros::Controller {
private:
    std::string rumble_queue;
    std::function<std::string()> display_functions[3] = {
        nullptr,
        nullptr,
        nullptr
    };
    std::set<int> active_lines;
    int interval_ms = 1000;
    bool rumble_on = false;
    int32_t prev_update_time = pros::millis();
    int32_t prev_rumble_time = pros::millis();
    int display_update_count = 0;
public:
    using pros::Controller::Controller;

    void update_display();
    void display(std::uint8_t line, std::function<std::string()> func);
    void set_text(std::uint8_t line, std::string text);
    void remove(std::uint8_t line);
    void rumble(std::string pattern);
    void toggle_rumble();
    void set_rumble(bool on);
    void set_rumble_interval(int interval_ms);
};

} // namespace miku