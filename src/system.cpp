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

    left_motors.tare_position_all();
    right_motors.tare_position_all();
    intake.tare_position_all();

    pros::Task display_task(display);

    // match = pros::competition::is_competition_switch();

    // while(!pros::competition::is_autonomous() || !pros::competition::is_field_control) {
    //     pros::delay(50);
    // }

    // Pose start = Pose(6.5, -48, M_PI / 2); // right sawp

    // // Pose start = Pose(24, -48, M_PI); // test
    // setPose(start);
    // initialize_mcl();

    // pros::Task flush_task([]() {
    //     while(true) {
    //         flush_logs();
    //         pros::delay(1000);
    //         // master.rumble(".");
    //     }
    // });

    // pros::Task autonomous_task([]() {
    //     while (true) {
    //         update_odom();
    //         update_particles();

    //         // log_mcl();

    //         Pose belief = get_pose_estimate();
    //         belief.theta = getPose().theta;
    //         setPose(belief);

    //         // log_mcl();

    //         resample_particles();

    //         pros::delay(10);
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