#pragma once

enum class NotificationType {
    FUCKED,
    IMPORTANT,
    WARNING,
    DISPLAY
};

struct DisplayItem;

void add_warning(const std::string& warning, NotificationType type);

void display();