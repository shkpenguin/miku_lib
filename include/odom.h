#pragma once

#include "misc.h"

Pose getPose(bool standard = false);
Pose getSpeed();
void setPose(Pose new_pose);

void update_odom();
void init_odom(Pose inital_pose);