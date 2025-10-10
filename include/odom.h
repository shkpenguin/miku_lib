#pragma once

#include "geometry.h"

void set_wheel_tracking(bool enabled);
bool get_wheel_tracking();

struct PoseSettings {
    bool standard = false; // if true, returns (x, y, 90 - theta) instead of (x, y, theta)
    bool degrees = false; // if true, theta is in degrees, else radians
};

Pose get_pose(PoseSettings settings = {false, false});
Pose get_speed();

void set_pose(Pose new_pose);

void update_odom();
void start_odom(Pose initial_pose);