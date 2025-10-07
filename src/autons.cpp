#include "mp.h"
#include "motions.h"
#include "main.h"
#include "mcl.h"
#include "autons.h"
#include "controller.h"
#include "util.h"
#include "gif-pros/gifclass.hpp"
#include "subsystems.h"
#include <vector>

pros::Task* autonomous_task = nullptr;
pros::Task* intake_task = nullptr;
pros::Task* controller_task = nullptr;

bool tracking = true;

void set_tracking(bool enabled) {
    tracking = enabled;
}

bool get_tracking() {
    return tracking;
}

std::vector<ControlPoint> test_cp = {
    {0, -48, 0},
    {0, -48, 20},
    {24, -48, 20},
    {48, -48, 20},
    {48, -48, 0}
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
    
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void test() {
    test_path.calculate_waypoints();
    ramsete(test_path.get_waypoints(), 10000);
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

std::vector<ControlPoint> right_9ball_cp_1 = {
    {8, -48, 0},
    {8, -48, 70},
    {23, -24, 40},
    {38, -14, 40},
    {48, -7, 20},
    {48, -7, 0}
};

std::vector<ControlPoint> right_9ball_cp_2 = {
    {48, -7, 0},
    {48, -7, 30},
    {40, -36, 40},
    {48, -48, 30},
    {48, -48, 0}
};

BezierPath right_9ball_1(right_9ball_cp_1);
BezierPath right_9ball_2(right_9ball_cp_2);

void pre_right_9ball() {

    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void right_9ball() {

    right_9ball_1.calculate_waypoints();
    right_9ball_2.calculate_waypoints();

    intake.move_voltage(12000);
    ramsete(right_9ball_1.get_waypoints(), 3000, {.cutoff = 3.0, .end_cutoff = 3.0});
    ramsete(right_9ball_2.get_waypoints(), 1800, {.reverse = true, .angular_weight = 0.01, .end_cutoff = 2.0});
    set_hood(true);
    intake.move_voltage(0);
    set_lock(true);
    // turn_heading(0, 500, {.cutoff = 10.0});
    // move_pose({49, -24, 0}, 1000);
    move_point({48, -26}, 1000);
    intake.move_voltage(12000);
    pros::delay(1200);
    move_pose({48, -60, 180}, 1200, {.async = true, .distance_weight = 2.5, .angular_weight = 2.5});
    pros::delay(200);
    set_lock(false);
    set_loading(true);
    wait_until_done();
    move_point({48, -65}, 1000);
    move_time(5000, 1000);
    intake.move_voltage(0);
    set_lock(true);
    move_pose({48, -28, 0}, 1800, {.async = true, .distance_weight = 2.5, .angular_weight = 2.0});
    pros::delay(200);
    set_loading(false);
    wait_until_done();
    intake.move_voltage(12000);
    pros::delay(700);
    intake.move_voltage(0);
    move_time(3000, 3000);

}

std::vector<ControlPoint> skills_cp_1 = {
    {-6, -48, 0},
    {-6, -48, 100},
    {-28, -48, 20},
    {-44, -54, 20},
    {-48, -64, 30},
    {-48, -64, 0}
};

BezierPath skills_1(skills_cp_1);

void pre_skills() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    set_hood(true);
    set_loading(true);
}

void skills() {
    intake.move_voltage(12000);
    ramsete(skills_1.get_waypoints(), 3000, {.angular_weight = 0.015, .end_cutoff = 3.0});
    // set_tracking(false);
    // move_time(4000, 1000); // uncomment once aligner is fixed
    // set_tracking(true);
    move_pose({-48, -32, 0}, 2000, {.async = true, .distance_weight = 2.3, .angular_weight = 2.3});
    pros::delay(200);
    set_loading(false);
    wait_until_done();
    set_lock(true);
    pros::delay(1200);
    set_lock(false);
    move_point({-48, -50}, 800, {.reverse = true});
    move_pose({-24, -24, 45}, 2000);
    move_point({-48, -48}, 2000, {.reverse = true});
}

std::vector<std::shared_ptr<BezierPath>> test_paths;
std::vector<std::shared_ptr<BezierPath>> right_sawp_paths;
std::vector<std::shared_ptr<BezierPath>> right_elims_paths;
std::vector<std::shared_ptr<BezierPath>> right_9ball_paths;
std::vector<std::shared_ptr<BezierPath>> skills_paths;
std::vector<Auton> autons;

void init_autons() {

    test_paths.clear();
    test_paths.push_back(std::shared_ptr<BezierPath>(&test_path, [](BezierPath*){}));

    right_sawp_paths.clear();
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_1, [](BezierPath*){})); 
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_2, [](BezierPath*){}));
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_3, [](BezierPath*){}));
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_4, [](BezierPath*){}));
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_5, [](BezierPath*){} ));
    right_sawp_paths.push_back(std::shared_ptr<BezierPath>(&right_sawp_6, [](BezierPath*){}));

    right_elims_paths.clear();
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_1, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_2, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_3, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_3_1, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_4, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_5, [](BezierPath*){}));
    right_elims_paths.push_back(std::shared_ptr<BezierPath>(&right_elims_6, [](BezierPath*){}));

    right_9ball_paths.clear();
    right_9ball_paths.push_back(std::shared_ptr<BezierPath>(&right_9ball_1, [](BezierPath*){}));

    skills_paths.clear();
    skills_paths.push_back(std::shared_ptr<BezierPath>(&skills_1, [](BezierPath*){}));

    // Create Autons dynamically
    autons.clear();
    autons.emplace_back("Test", setup, test, Pose(24, -48, 90, false), test_paths);
    autons.emplace_back("Right Sawp", pre_right_sawp, right_sawp, Pose(6.5, -48, M_PI/2), right_sawp_paths);
    autons.emplace_back("Right Elims", pre_right_elims, right_elims, Pose(18, -53, M_PI/6), right_elims_paths);
    autons.emplace_back("Right 9 Ball", pre_right_9ball, right_9ball, Pose(8, -48, 30, false), right_9ball_paths);
    autons.emplace_back("Skills", pre_skills, skills, Pose(-6, -48, -90, false), skills_paths);

}

void initialize() {

    //pros::lcd::initialize();
    left_motors.tare_position_all();
    right_motors.tare_position_all();
    intake.tare_position_all();

    master.set_text(0, 0, "IMU Calibrating");

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

    master.rumble("..");

    master.set_text(0, 0, "              ");

    intake_task = new pros::Task(intake_control);

    selected_index = 3;

    init_autons();

    if (autons.empty()) return;
    //display_selector();

    Auton& selected_auton = autons[selected_index];
    selected_auton.pre_auton();
    for(auto& path : selected_auton.paths) {
        path->calculate_waypoints();
    }

    initialize_pose(selected_auton.start_pose);

    controller_task = new pros::Task(display_controller);

}

void autonomous() { 
    
    #if LOGGING_ENABLED
    file.open("log.txt");
    #endif

    static Gif gif("/usd/miku.gif", lv_scr_act());
    // intake_task = new pros::Task(intake_control);

    autonomous_task = new pros::Task([]() {
        while (true) {
            #if LOGGING_ENABLED
            update_odom();
            log_mcl();

            update_particles();
            log_mcl();
            #else
            if(tracking) update_odom();
            update_particles();
            #endif

            Pose belief = get_pose_estimate();
            belief.theta = get_pose().theta;
            set_pose(belief);

            resample_particles();

            pros::delay(10);
        }
    });

    #if LOGGING_ENABLED
    pros::Task log_task = pros::Task([]() {
        while (true) {
            flush_logs();
            pros::delay(1000);
        }
    });
    #endif

    Auton& selected_auton = autons[selected_index];
    selected_auton.auton(); 

}