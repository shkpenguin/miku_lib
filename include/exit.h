#pragma once

/**
 * @param range The acceptable range of error
 * @param time The time duration to check for the exit condition(ms)
 */
struct RangeExit {
    double range;
    int time;
    int startTime = -1;
    bool done = false;

    RangeExit(const double range, const int time);

    bool get_exit();
    bool update(const double input);
    void reset();
};