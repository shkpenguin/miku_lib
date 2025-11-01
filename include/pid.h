#pragma once

#include <vector>

struct Gains {
    double kP;
    double kI;
    double kD;
    double windup_range = 0.0;

    Gains(double kP, double kI, double kD, double windup_range = 0.0) : kP(kP), kI(kI), kD(kD), windup_range(windup_range) {}
    Gains() : kP(0.0), kI(0.0), kD(0.0), windup_range(0.0) {}
};

/**
 * @param Gains gains The PID gains
 * @param double windup_range The range at which to limit the integral term (default 0, no limit)
 * @param bool sign_flip_reset Whether to reset the integral term when the error changes sign (default true)
 * @param bool trapezoidal Whether to use trapezoidal integration for the integral term (default true)
*/
class PID {
    public:
        PID(double kP, double kI, double kD, double windup_range = 0, bool sign_flip_reset = true, bool trapezoidal = true);
        PID(Gains gains, bool sign_flip_reset = true, bool trapezoidal = true)
            : PID(gains.kP, gains.kI, gains.kD, gains.windup_range, sign_flip_reset, trapezoidal) {}
        PID() : PID(0.0, 0.0, 0.0) {}

        double update(double error);

        void set_gains(Gains gains);
        void set_kp(double p);
        void set_ki(double i);
        void set_kd(double d);

        void reset();

        PID& operator=(const PID& other);
    protected:
        // gains
        double kP;
        double kI;
        double kD;

        // optimizations
        const double windup_range;
        const bool sign_flip_reset;
        const bool trapezoidal;

        double integral = 0;
        double prevError = 0;
};