#pragma once

extern pros::Task* intake_task;

extern double target_vel;

extern bool hood_up;
extern bool lock;
extern bool loading;
extern bool descore;

// Functions
void set_intake_tbh(bool enabled);
bool get_intake_tbh();
void intake_control();

// @param state true = closed, false = open
void set_lock(bool closed);

// @param state true = down, false = up
void set_loading(bool down);

// @param state true = up, false = down
void set_hood(bool up);

// @param state true = down, false = up
void set_descore(bool down);

bool get_hood();
bool get_loading();
bool get_lock();
bool get_descore();

void set_intake_velocity(double vel);