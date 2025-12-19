#pragma once

#include <deque>
#include "miku/motions.h"
#include "api.h"

extern pros::Task* autonomous_system_task;
extern std::deque<MotionPrimitive*> motion_queue;
extern pros::Mutex queue_mutex;
extern MotionPrimitive* current_motion;

void queue_motion(MotionPrimitive* motion);
void queue_after_current(MotionPrimitive* motion);