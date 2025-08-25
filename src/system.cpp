#include "main.h"
#include "gif-pros/gifclass.hpp"
#include "config.h"
#include "odom.h"
#include "api.h"
#include "mcl.h"
#include "misc.h"

bool match;

void system();

void initialize() {
	static Gif gif("/usd/miku.gif", lv_scr_act());
	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10); // Wait for IMU calibration
	}

    match = pros::competition::is_competition_switch();

    system();
}

void system() {
    
    // disabled

    // auton selector

    init_odom(Pose(0, 0, 0));
    initialize_particles();

    // wait for autonomous

    while(!pros::competition::is_autonomous()) {
        pros::delay(50);
    }

    pros::Task autonomous_task([]() {
        while (true) {
            update_odom();
            update_particles();

            setPose(get_pose_estimate());

            resample_particles();

            pros::delay(10);
        }
    });

}