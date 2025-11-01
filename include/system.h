#pragma once

#include "motions.h"

void queue_motion(std::shared_ptr<MotionPrimitive> motion);

void queue_after_current(std::shared_ptr<MotionPrimitive> motion);