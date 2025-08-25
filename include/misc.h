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
    float kP;
    float kI;
    float kD;

    Gains(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}
};

class PID {
    public:

        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false, bool trapezoidal = true);

        float update(float error);

        void setGains(Gains gains);

        void reset();
    protected:
        // gains
        float kP;
        float kI;
        float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;
        const bool trapezoidal;

        float integral = 0;
        float prevError = 0;
};