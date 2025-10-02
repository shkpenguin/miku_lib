#include "mp.h"
#include "motions.h"
#include "main.h"
#include "mcl.h"
#include "misc.h"
#include "autons.h"
#include "notif.h"
#include "util.h"
#include "gif-pros/gifclass.hpp"
#include <vector>

pros::Task* autonomous_task = nullptr;
pros::Task* controller_display = nullptr;

std::vector<ControlPoint> test_cp = {
    {24, -48, 0},
    {24, -24, 20},
    // {24, 24, 30},
    {0, 48, 20},
    {0, 48, 20}
};

std::vector<ControlPoint> right_elims_cp_1 = {
    {18, -51, 0},
    {18, -51, 100},
    {27, -45, 100},
    {32, -35, 35},
    {36, -24, 25},
    {41, -15, 15},
    {44, -8, 0},
    {44, -8, 0}
};

std::vector<ControlPoint> right_elims_cp_2 = {
    {43, -15, 0},
    {43, -15, 100},
    {32, -16, 50},
    {22, -25, 0},
    {22, -25, 0}
};

std::vector<ControlPoint> right_elims_cp_3 = {
    {22, -25, 0},
    {22, -25, 100},
    {28, -38, 40},
    {42, -46, 30},
    {48, -48, 0},
    {48, -48, 0}
};

std::vector<ControlPoint> right_elims_cp_3_1 = {
    {42, -42, 0},
    {42, -42, 30},
    {48, -42, 0},
    {48, -42, 0}
};

std::vector<ControlPoint> right_elims_cp_4 = {
    {48, -42, 0},
    {48, -38, 70},
    {48, -26, 20},
    {48, -26, 0}
};

std::vector<ControlPoint> right_elims_cp_5 = {
    {48, -42, 30},
    {48, -42, 30},
    {48, -64, 30},
    {48, -64, 0}
};

std::vector<ControlPoint> right_elims_cp_6 = {
    {48, -60, 0},
    {48, -60, 75},
    {48, -34, 0},
    {48, -34, 0}
};

std::vector<ControlPoint> right_sawp_cp_1 = {
    {6.5, -48, 0},
    {6.5, -48, 100},
    {24, -48, 80},
    {42, -48, 15},
    {48, -48, 0},
    {48, -48, 0}
};

std::vector<ControlPoint> right_sawp_cp_2 = {
    {48, -48, 40},
    {48, -48, 40},
    {48, -64, 30},
    {48, -64, 0}
};

std::vector<ControlPoint> right_sawp_cp_3 = {
    {48, -60, 0},
    {48, -60, 90},
    {48, -34, 0},
    {48, -34, 0}
};

std::vector<ControlPoint> right_sawp_cp_4 = {
    {48, -36, 0},
    {48, -36, 40},
    {24, -24, 20},
    {18, -18, 20},
    {15, -15, 0},
    {15, -15, 0}
};

std::vector<ControlPoint> right_sawp_cp_5 = {
    {20, -24, 0},
    {20, -24, 100},
    {0, -21, 50},
    {-12, -21, 15},
    {-24, -21, 0},
    {-24, -21, 0}
};

std::vector<ControlPoint> right_sawp_cp_6 = {
    {-24, -21, 0},
    {-24, -21, 50},
    {-18, -18, 20},
    {-14, -14, 0},
    {-14, -14, 0}
};

BezierPath test_path(test_cp);
std::vector<Waypoint> test_waypoints;

BezierPath right_elims_1(right_elims_cp_1);
BezierPath right_elims_2(right_elims_cp_2);
BezierPath right_elims_3(right_elims_cp_3);
BezierPath right_elims_3_1(right_elims_cp_3_1);
BezierPath right_elims_4(right_elims_cp_4);
BezierPath right_elims_5(right_elims_cp_5);
BezierPath right_elims_6(right_elims_cp_6);

BezierPath right_sawp_1(right_sawp_cp_1);
BezierPath right_sawp_2(right_sawp_cp_2);
BezierPath right_sawp_3(right_sawp_cp_3);
BezierPath right_sawp_4(right_sawp_cp_4);
BezierPath right_sawp_5(right_sawp_cp_5);
BezierPath right_sawp_6(right_sawp_cp_6);

void setup() {
    turn_heading(30, 10000);
}

void test() {
    move_pose(Pose(0, 48, 0), 3000, { .angular_weight = 1 });
    // move_point(Point(0, 48), 3000);
    master.rumble("...");
}

void pre_right_sawp() {

    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);
    set_hood(true);

}

void right_sawp() {

    right_sawp_1.calculate_waypoints();
    right_sawp_2.calculate_waypoints();
    right_sawp_3.calculate_waypoints();
    right_sawp_4.calculate_waypoints();
    right_sawp_5.calculate_waypoints();
    right_sawp_6.calculate_waypoints();
    std::vector<Waypoint> right_sawp_1_waypoints = right_sawp_1.get_waypoints();
    std::vector<Waypoint> right_sawp_2_waypoints = right_sawp_2.get_waypoints();
    std::vector<Waypoint> right_sawp_3_waypoints = right_sawp_3.get_waypoints();
    std::vector<Waypoint> right_sawp_4_waypoints = right_sawp_4.get_waypoints();
    std::vector<Waypoint> right_sawp_5_waypoints = right_sawp_5.get_waypoints();
    std::vector<Waypoint> right_sawp_6_waypoints = right_sawp_6.get_waypoints();

    // intake.move_voltage(12000);

    // ramsete(right_sawp_1_waypoints, 3000, false, true, 2.0);
    // wait_until_done();
    // pros::delay(300);
    // set_loading(true);
    // turn_heading(180, 1200);
    // ramsete(right_sawp_2_waypoints, 600, false, false, 3.0, 500);
    // pros::delay(400);
    // move_time(-4000, 400);
    // move_time(0, 200);
    // turn_heading(0, 1000);
    // set_loading(false);

    // set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    // ramsete(right_sawp_3_waypoints, 600, false, true);
    // wait_until_within({48, -34}, 6.0);
    // set_lock(true);
    // wait_until_done();
    // pros::delay(1200);
    // set_lock(false);
    // set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    // move_time(-6000, 300);
    // turn_heading(-40, 700);
    // ramsete(right_sawp_4_waypoints, 2200);
    // intake.move_voltage(-8000);
    // pros::delay(1500);
    // intake.move_voltage(12000);
    // move_time(-6000, 200);

    // turn_heading(-90, 700);
    // ramsete(right_sawp_5_waypoints, 2000);
    // turn_heading(50, 700);
    // intake.move_velocity(4000);
    // ramsete(right_sawp_6_waypoints, 2000);
    // move_time(2000, 300);
    // set_hood(false);
    // set_lock(true);
    // move_time(2000, 800);

}

void pre_right_elims() {

    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

}

void right_elims() {

    right_elims_1.calculate_waypoints();
    right_elims_2.calculate_waypoints();
    right_elims_3.calculate_waypoints();
    right_elims_3_1.calculate_waypoints();
    right_elims_4.calculate_waypoints();
    right_elims_5.calculate_waypoints();
    right_elims_6.calculate_waypoints();
    std::vector<Waypoint> right_elims_1_waypoints = right_elims_1.get_waypoints();
    std::vector<Waypoint> right_elims_2_waypoints = right_elims_2.get_waypoints();
    std::vector<Waypoint> right_elims_3_waypoints = right_elims_3.get_waypoints();
    std::vector<Waypoint> right_elims_3_1_waypoints = right_elims_3_1.get_waypoints();
    std::vector<Waypoint> right_elims_4_waypoints = right_elims_4.get_waypoints();
    std::vector<Waypoint> right_elims_5_waypoints = right_elims_5.get_waypoints();
    std::vector<Waypoint> right_elims_6_waypoints = right_elims_6.get_waypoints();
    intake.move_voltage(12000);

    // ramsete(right_elims_1_waypoints, 3000, false, true, 2.0);
    // // // wait_until_within({48, -6}, 2.0); // elims block
    // // // intake.move_voltage(0);
    // wait_until_done();
    // move_time(-6000, 100);
    // turn_heading(-120, 1000);
    // intake.move_voltage(12000);
    // set_hood(true);
    // ramsete(right_elims_2_waypoints, 1500);
    // wait_until_done();
    // turn_heading(150, 700);
    // ramsete(right_elims_3_waypoints, 2000, false, false, 3.0);
    // turn_heading(0, 1000);
    // set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    // ramsete(right_elims_4_waypoints, 1200, false, true);
    // wait_until_within({48, -34}, 5.0);
    // set_lock(true);
    // wait_until_done();
    // pros::delay(1300);
    // set_lock(false);
    // set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    // move_time(-4000, 400);
    // move_time(0, 300);
    // set_loading(true);
    // turn_heading(180, 1000);
    // ramsete(right_elims_5_waypoints, 600, false, false, 3.0, 200);
    // pros::delay(600);
    // move_time(-4000, 400);
    // move_time(0, 300);
    // turn_heading(0, 1000);
    // set_loading(false);

    // set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    // ramsete(right_elims_6_waypoints, 600, false, true);
    // wait_until_within({48, -34}, 5.0);
    // set_lock(true);
    // wait_until_done();

}

std::vector<std::shared_ptr<BezierPath>> right_sawp_paths;
std::vector<std::shared_ptr<BezierPath>> right_elims_paths;
std::vector<Auton> autons;

void init_autons() {

    // Create Bezier paths dynamically
    right_sawp_paths.clear();
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_1));
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_2));
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_3));
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_4));
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_5));
    right_sawp_paths.push_back(std::make_shared<BezierPath>(right_sawp_6));

    right_elims_paths.clear();
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_1));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_2));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_3));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_3_1));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_4));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_5));
    right_elims_paths.push_back(std::make_shared<BezierPath>(right_elims_6));

    // Create Autons dynamically
    autons.clear();
    autons.emplace_back("Right Sawp", pre_right_sawp, right_sawp, Pose(6.5, -48, M_PI/2), right_sawp_paths);
    autons.emplace_back("Right Elims", pre_right_elims, right_elims, Pose(18, -53, M_PI/6), right_elims_paths);
}

/*
void initialize() {

    selected_index = 1;

    controller_display = new pros::Task(display_controller);

    static Gif gif("/usd/miku.gif", lv_scr_act());

	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10); // Wait for IMU calibration
	}

    left_motors.tare_position_all();
    right_motors.tare_position_all();
    intake.tare_position_all();

    init_autons();

    Auton& selected_auton = autons[selected_index];
    selected_auton.pre_auton();
    // for(auto& path : selected_auton.paths) {
    //     path->calculate_waypoints();
    // }

    // pros::Task brain_display(display_selector);

}

void autonomous() {

    if (autons.empty()) return;

    Auton& selected_auton = autons[selected_index];

    Pose start = selected_auton.start_pose;
    setPose(start);
    initialize_mcl();

    autonomous_task = new pros::Task([]() {
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

    selected_auton.auton();  // only run if non-null
}
*/

void initialize() {
    controller_display = new pros::Task(display_controller);

    static Gif gif("/usd/miku.gif", lv_scr_act());

    /*
    left_motors.tare_position_all();
    right_motors.tare_position_all();
    intake.tare_position_all();

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10); // Wait for IMU calibration
	}

    setPose(Pose(24, -48, M_PI / 2));
    initialize_mcl();

    autonomous_task = new pros::Task([]() {
        while (true) {
            update_odom();
            log_mcl();

            update_particles();
            log_mcl();

            Pose belief = get_pose_estimate();
            belief.theta = getPose().theta;
            setPose(belief);

            resample_particles();

            pros::delay(10);
        }
    });

    pros::Task log_task = pros::Task([]() {
        while (true) {
            flush_logs();
            pros::delay(1000);
        }
    });

    pros::delay(10);

    */

}

void autonomous() {

    test();

}