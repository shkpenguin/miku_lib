#pragma once

#include <vector>
#include "config.h"
#include "util.h"
#include "mp.h"

double voltage_lookup(double velocity);

struct Point {
    double x;
    double y;

    Point(double x = 0.0, double y = 0.0) 
        : x(x), y(y) {}
};

struct Pose {
    double x; 
    double y;
    double theta; 

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0) 
        : x(x), y(y), theta(theta) {}
    Pose(Point position, double theta = 0.0) 
        : x(position.x), y(position.y), theta(theta) {}
};

struct Particle {
    Pose pose; 
    double weight; 
};

struct Gains {
    double kP;
    double kI;
    double kD;

    Gains(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}
};

class PID {
    public:
        PID(double kP, double kI, double kD, double windupRange = 0, bool signFlipReset = false, bool trapezoidal = true);
        PID(Gains gains, double windupRange = 0, bool signFlipReset = false, bool trapezoidal = true)
            : PID(gains.kP, gains.kI, gains.kD, windupRange, signFlipReset, trapezoidal) {}

        double update(double error);

        void setGains(Gains gains);

        void reset();
    protected:
        // gains
        double kP;
        double kI;
        double kD;

        // optimizations
        const double windupRange;
        const bool signFlipReset;
        const bool trapezoidal;

        double integral = 0;
        double prevError = 0;
};

/**
 * @param range The acceptable range of error
 * @param time The time duration to check for the exit condition(ms)
 */
struct ExitCondition {
    double range;
    int time;
    int startTime = -1;
    bool done = false;

    ExitCondition(const double range, const int time);

    bool getExit();
    bool update(const double input);
    void reset();
};

enum Side {
    LEFT,
    RIGHT
};