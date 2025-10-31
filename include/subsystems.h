#pragma once

extern pros::Task* intake_task;

extern double target_vel;

// Functions
void set_intake_tbh(bool enabled);
bool get_intake_tbh();
void intake_control();

void set_intake_velocity(double vel);