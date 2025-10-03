#include "main.h"

#include "config.h"
#include "subsystems.h"

double target_vel = 0;

PID intake_pid(intake_gains, 1000, true, false);

void intake_control() {

    while(true) {

        double intake_vel = intake.get_average_velocity();
        double error = target_vel - intake_vel;
        double output = intake_pid.update(error);

        intake.move_voltage(output);
        
        pros::delay(20);

    }

}