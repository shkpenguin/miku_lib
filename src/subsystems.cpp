#include "main.h"
#include "config.h"
#include "subsystems.h"
#include "lut.h"

double target_vel = 400;

double gain = 1.28125; // 1.25 | 1.28125
double output = 0;
double prev_error = 0;
double tbh = output;
bool tbh_enabled = false;

void set_intake_tbh(bool enabled) {
    tbh_enabled = enabled;
}

bool get_intake_tbh() {
    return tbh_enabled;
}

void intake_control() {

    while(true) {

        if(!tbh_enabled) {
            pros::delay(100);
            continue;
        }
        double intake_vel = intake.get_average_velocity();
        master.print(1,0,"vel: %f",intake_vel);
        double error = target_vel - intake_vel;
        output += error * gain;

        if (std::signbit(error)!=std::signbit(prev_error)) {
            output = 0.5 * (output + tbh);
            tbh = output;
        }
        prev_error = error;

        intake.move_voltage(output);
        
        pros::delay(20);

    }

}

void set_intake_velocity(double vel) {
    tbh_enabled = true;
    target_vel = vel;
    output = intake_lut.get_value(vel);
    prev_error = 0;
    tbh = output;
}