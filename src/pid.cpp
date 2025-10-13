#include "util.h"
#include "pid.h"
#include "api.h"

PID::PID(double kP, double kI, double kD, double windupRange, bool signFlipReset, bool trapezoidal)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset),
      trapezoidal(trapezoidal) {}

double PID::update(const double error) {

    // calculate derivative
    const double derivative = error - prevError;

    // calculate integral
    if (trapezoidal) {
        if (sign(derivative) != sign(error)) {
            integral += (error + prevError) / 2;
        } else {
            integral += error;
        }
    } else {
        integral += error;
    }
    if (sign(error) != sign((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;
    
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void PID::setGains(Gains gains) {
    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
}

void PID::setKp(double p) {
    kP = p;
}

void PID::setKi(double i) {
    kI = i;
}

void PID::setKd(double d) {
    kD = d;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

ExitCondition::ExitCondition(const double range, const int time)
    : range(range),
      time(time) {}

bool ExitCondition::getExit() { return done; }

bool ExitCondition::update(const double input) {
    const int curTime = pros::millis();
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + time) done = true;
    return done;
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
}