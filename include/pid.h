#pragma once

#include <vector>

struct Gains {
    double kP;
    double kI;
    double kD;

    Gains(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}
};

/**
 * @param Gains gains The PID gains
 * @param double windupRange The range at which to limit the integral term (default 0, no limit)
 * @param bool signFlipReset Whether to reset the integral term when the error changes sign (default false)
 * @param bool trapezoidal Whether to use trapezoidal integration for the integral term (default true)
*/
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