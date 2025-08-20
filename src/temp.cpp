#include "api.h"
#include "config.h"
#include "notif.h"

struct MotorTemp {
    pros::Motor motor;
    std::string name;
    int prev_temperature = 0;
    MotorTemp(pros::Motor m, std::string n) : motor(m), name(n) {}
};

std::vector<MotorTemp> motor_temps = {
    MotorTemp(left_front, "LF Motor"),
    MotorTemp(left_middle, "LM Motor"),
    MotorTemp(left_back, "LB Motor"),
    MotorTemp(right_front, "RF Motor"),
    MotorTemp(right_middle, "RM Motor"),
    MotorTemp(right_back, "RB Motor"),
    MotorTemp(bottom_intake, "Bottom Intake"),
    MotorTemp(top_intake, "Top Intake"),
};

void checkTemperatures() {
    for (auto& motor_temp : motor_temps) {
        int temp = motor_temp.motor.get_temperature();
        if(motor_temp.prev_temperature < 55 & temp > 55) {
            add_warning(motor_temp.name + " is overheated!", NotificationType::WARNING);
        }
        if(motor_temp.prev_temperature < 62 & temp > 62) {
            add_warning(motor_temp.name + " is very overheated!", NotificationType::IMPORTANT);
        }
        motor_temp.prev_temperature = temp;
    }
}