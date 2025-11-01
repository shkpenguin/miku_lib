#pragma once
#include "time.h"

enum class NotificationType {
    FUCKED,
    IMPORTANT,
    WARNING,
    DISPLAY
};

extern Timer rumble_timer;

struct DisplayItem;

void add_warning(const std::string& warning, NotificationType type);

void display_controller();
extern pros::Task* controller_task;