#include "exit.h"
#include "api.h"
#include <cmath>

RangeExit::RangeExit(const double range, const int time)
    : range(range),
      time(time) {}

bool RangeExit::get_exit() { return done; }

bool RangeExit::update(const double input) {
    const int curTime = pros::millis();
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + time) done = true;
    return done;
}

void RangeExit::reset() {
    startTime = -1;
    done = false;
}

