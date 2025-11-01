#include "util.h"
#include "pid.h"
#include "api.h"

PID::PID(double kP, double kI, double kD, double windup_range, bool sign_flip_reset, bool trapezoidal)
    : kP(kP),
      kI(kI),
      kD(kD),
      windup_range(windup_range),
      sign_flip_reset(sign_flip_reset),
      trapezoidal(trapezoidal) {}

double PID::update(const double error) {

    // calculate derivative
    const double derivative = error - prevError;

    // calculate integral
    if (trapezoidal) {
        integral += derivative / 2;
    }
    if (sign(error) != sign((prevError)) && sign_flip_reset) integral = 0;
    if (fabs(error) > windup_range && windup_range != 0) integral = 0;
    
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
    
}

PID& PID::operator=(const PID& other) {
    if (this != &other) {
        kP = other.kP;
        kI = other.kI;
        kD = other.kD;
        integral = other.integral;
        prevError = other.prevError;
    }
    return *this;
}

void PID::set_gains(Gains gains) {
    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
}

void PID::set_kp(double p) {
    kP = p;
}

void PID::set_ki(double i) {
    kI = i;
}

void PID::set_kd(double d) {
    kD = d;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}