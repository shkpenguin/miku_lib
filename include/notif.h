#pragma once
#include "timer.h"

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