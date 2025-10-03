#pragma once

#include "misc.h"

Pose get_pose(bool standard = false);
Pose get_speed();
void set_pose(Pose new_pose);

void update_odom();
void init_odom(Pose inital_pose);