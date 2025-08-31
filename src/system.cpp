#include "main.h"
#include "gif-pros/gifclass.hpp"
#include "config.h"
#include "odom.h"
#include "api.h"
#include "mcl.h"
#include "misc.h"
#include "notif.h"

bool match;

void system();

void initialize() {
	static Gif gif("/usd/miku.gif", lv_scr_act());
	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10); // Wait for IMU calibration
	}

    match = pros::competition::is_competition_switch();

    pros::Task autonomous_task([]() {
        setPose({24, -48, 0});
        initialize_particles();

        while (true) {
            update_odom();
            update_particles();

            Pose belief = get_pose_estimate();
            belief.theta = getPose().theta;
            setPose(belief);

            resample_particles();

            pros::delay(10);
        }
    });

    pros::Task flush_task([]() {
        while(true) {
            flush_logs();
            pros::delay(1000);
            // master.rumble(".");
        }
    });
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