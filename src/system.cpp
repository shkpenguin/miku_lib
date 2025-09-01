#include "main.h"
#include "gif-pros/gifclass.hpp"
#include "config.h"
#include "odom.h"
#include "api.h"
#include "mcl.h"
#include "misc.h"
#include "notif.h"

// bool match;

// void system();

void initialize() {
	static Gif gif("/usd/miku.gif", lv_scr_act());
	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10); // Wait for IMU calibration
	}

    pros::Task display_task(display);

    // match = pros::competition::is_competition_switch();

    // while(!pros::competition::is_autonomous() || !pros::competition::is_field_control) {
    //     pros::delay(50);
    // }

    // pros::Task flush_task([]() {
    //     while(true) {
    //         flush_logs();
    //         pros::delay(1000);
    //         // master.rumble(".");
    //     }
    // });
}

// void system() {

    // disabled

//     // auton selector

//     init_odom(Pose(24, -48, 0));
//     initialize_particles();

//     // wait for autonomous

//     if(match) {
//         while(!pros::competition::is_autonomous()) {
//             pros::delay(50);
//         }
//     }

    

// }