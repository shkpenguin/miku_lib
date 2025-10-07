#include "main.h"

#include "config.h"
#include "subsystems.h"
#include "lut.h"

double target_vel = 400;

double gain = 1.28;//1.25 | 1.28125
double output = 6000;
double prev_error = 0;
double tbh = output;
bool tbh_enabled = false;

void set_intake_tbh(bool enabled) {
    tbh_enabled = enabled;
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
    target_vel = vel;
    output = intake_lut.get_voltage(vel);
    prev_error = 0;
    tbh = output;
}

bool hood_up = false;
bool lock = false;
bool loading = false;
bool descore = false;

// @param state true = open, false = closed
void set_lock(bool open) {
    lock = open;
    lock_piston.set_value(lock);
}

// @param state true = active, false = inactive
void set_loading(bool active) {
    loading = active;
    loader_piston.set_value(loading);
}

// @param state true = up, false = down
void set_hood(bool up) {
    hood_up = up;
    hood_piston.set_value(hood_up);
}

// 
void set_descore(bool state) {
    descore = state;
    descore_piston.set_value(descore);
}

bool get_hood() {
    return hood_up;
}

bool get_loading() {
    return loading;
}

bool get_lock() {
    return lock;
}

bool get_descore() {
    return descore;
}