#pragma once

#include "miku/motions.h"

void queue_motion(MotionPrimitive* motion);

void queue_after_current(MotionPrimitive* motion);

void system_task(void*);