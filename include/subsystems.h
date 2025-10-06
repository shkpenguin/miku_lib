#pragma once

#include "config.h"

extern pros::Task* intake_task;

extern double target_vel;

extern bool hood_up;
extern bool lock;
extern bool loading;
extern bool descore;

// Functions
void set_intake_tbh(bool enabled);
void intake_control();

void set_lock(bool state);
void set_loading(bool state);
void set_hood(bool state);
void set_descore(bool state);

bool get_hood();
bool get_loading();
bool get_lock();
bool get_descore();

void set_intake_velocity(double vel);