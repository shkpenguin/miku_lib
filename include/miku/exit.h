#pragma once
#include "pros/rtos.hpp"
#include "geometry.h"

/**
 * @param range The acceptable range of error
 * @param time The time duration to check for the exit condition(ms)
 */
struct RangeExit {
    double range;
    int time;
    int start_time = -1;
    bool done = false;

    RangeExit(const double range, const int time) : range(range), time(time) {}

    bool get_exit() { return done; }
    bool update(const double input) {
        const int curr_time = pros::millis();
        if (std::fabs(input) > range) start_time = -1;
        else if (start_time == -1) start_time = curr_time;
        else if (curr_time >= start_time + time) done = true;
        return done;
    };
    void reset() { done = false; start_time = -1; }
};

struct PatienceExit {
    double total_deviation = 0;
    double max_deviation = 0;
    double patience = 0;
    double max_patience = 0;
    bool done = false;
    PatienceExit(double max_deviation, double max_patience)
        : max_deviation(max_deviation), max_patience(max_patience) {};

    bool get_exit() { return done; }
    bool update(const double input) {
        total_deviation += input;
        if (total_deviation >= max_deviation) {
            total_deviation = 0;
            patience += 1;
        }
        if (patience >= max_patience) done = true;
        return done;
    };
    void reset() {
        total_deviation = 0;
        max_deviation = 0;
        patience = 0;
        done = false;
    }

};