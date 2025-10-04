#include "main.h"

#include "config.h"
#include "subsystems.h"

double target_vel = 0;

PID intake_pid(intake_gains, 1000, true);

void intake_control() {

    while(true) {

        double intake_vel = intake.get_average_velocity() / 6;
        double error = target_vel - intake_vel;
        double output = intake_pid.update(error);

        intake.move_voltage(output);
        
        pros::delay(20);

    }

}

bool hood_up = false;
bool lock = false;
bool loading = false;
bool descore = false;

void intake_control();

void set_lock(bool state) {
    lock = state;
    lock_piston.set_value(lock);
}

void set_loading(bool state) {
    loading = state;
    loader_piston.set_value(loading);
}

void set_hood(bool state) {
    hood_up = state;
    hood_piston.set_value(hood_up);
}

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

void set_intake_velocity(double vel) {
    target_vel = vel;
}