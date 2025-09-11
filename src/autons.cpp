#include "mp.h"
#include "motions.h"
#include "main.h"
#include "mcl.h"
#include <vector>

std::vector<ControlPoint> test_cp = {
    {24, -48, 0},
    {24, -24, 20},
    // {24, 24, 30},
    {0, 48, 20},
    {0, 48, 20}
};

std::vector<ControlPoint> right_elims_cp_1 = {
    {18, -53, 0},
    {18, -53, 100},
    {23, -45, 100},
    {26, -35, 50},
    {34, -24, 25},
    {46, -8, 0},
    {46, -8, 0}
};

std::vector<ControlPoint> right_elims_cp_2 = {
    {47, -8, 0},
    {47, -8, 100},
    {35, -16, 50},
    {24, -24, 0},
    {24, -24, 0}
};

std::vector<ControlPoint> right_elims_cp_3 = {
    {24, -24, 0},
    {24, -24, 100},
    {34, -31, 100},
    {54, -38, 30}, // fucked point
    {54, -38, 0}
};

std::vector<ControlPoint> right_elims_cp_4 = {
    {48, -38, 0},
    {48, -38, 60},
    {48, -26, 0},
    {48, -26, 0}
};

std::vector<ControlPoint> right_elims_cp_5 = {
    {48, -26, 70},
    {48, -26, 70},
    {48, -64, 50},
    {48, -64, 0}
};

std::vector<ControlPoint> right_elims_cp_6 = {
    {48, -60, 0},
    {48, -60, 0},
    {48, -28, 50},
    {48, -28, 0}
};

std::vector<ControlPoint> right_sawp_cp_1 = {
    {6.5, -48, 0},
    {6.5, -48, 100},
    {24, -48, 80},
    {36, -48, 20},
    {48, -48, 0},
    {48, -48, 0}
};

std::vector<ControlPoint> right_sawp_cp_2 = {
    {48, -48, 50},
    {48, -48, 50},
    {48, -64, 50},
    {48, -64, 0}
};

std::vector<ControlPoint> right_sawp_cp_3 = {
    {48, -60, 0},
    {48, -60, 50},
    {48, -32, 0},
    {48, -32, 0}
};

std::vector<ControlPoint> right_sawp_cp_4 = {
    {48, -36, 0},
    {48, -36, 50},
    {24, -24, 20},
    {18, -18, 30},
    {14, -14, 0},
    {14, -14, 0}
};

std::vector<ControlPoint> right_sawp_cp_5 = {
    {20, -24, 0},
    {20, -24, 100},
    {0, -20, 40},
    {-12, -20, 15},
    {-24, -20, 0},
    {-24, -20, 0}
};

std::vector<ControlPoint> right_sawp_cp_6 = {
    {-24, -20, 0},
    {-24, -20, 40},
    {-10, -12, 10},
    {-10, -12, 0}
};

BezierPath test_path(test_cp);
std::vector<Waypoint> test_waypoints;

BezierPath right_elims_1(right_elims_cp_1);
BezierPath right_elims_2(right_elims_cp_2);
BezierPath right_elims_3(right_elims_cp_3);
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

    // pros::delay(500);
    // initialize_mcl({24, -48, 0});
    // test_path.calculate_waypoints();
    // test_waypoints = test_path.get_waypoints();
    // ramsete(test_waypoints, 20000);
    
    // // turn_heading(90, 10000);
    // move_point({0, -24}, 10000);
    move_point({24, -24}, 10000, true);

}

void right_sawp() {

    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

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

    intake.move_voltage(12000);

    ramsete(right_sawp_1_waypoints, 3000, false, true, 8.0);
    pros::delay(500);
    hood_piston.set_value(true);
    wait_until_done();
    loader_piston.set_value(true);
    // turn_point({48, -64}, 1200);
    turn_heading(180, 1000);
    ramsete(right_sawp_2_waypoints, 600, false, false, 3.0, 500);
    pros::delay(500);
    move_time(-6000, 300);
    loader_piston.set_value(false);
    turn_point({48, -24}, 1000);

    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    ramsete(right_sawp_3_waypoints, 600, false, true);
    wait_until_within({48, -36}, 3.0);
    lock_piston.set_value(true);
    wait_until_done();
    pros::delay(800);
    lock_piston.set_value(false);
    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    move_time(-6000, 300);
    turn_point({24, -24}, 1000);
    ramsete(right_sawp_4_waypoints, 2200);
    intake.move_voltage(-8000);

    pros::delay(1500);
    intake.move_voltage(12000);
    move_time(-6000, 200);

    turn_heading(-90, 1000);
    ramsete(right_sawp_5_waypoints, 2000);
    turn_point({-15, -15}, 1000);
    ramsete(right_sawp_6_waypoints, 1000);
    hood_piston.set_value(false);
    lock_piston.set_value(true);
    move_time(2000, 1000);

}

void right_elims() {
    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    right_elims_1.calculate_waypoints();
    right_elims_2.calculate_waypoints();
    right_elims_3.calculate_waypoints();
    right_elims_4.calculate_waypoints();
    right_elims_5.calculate_waypoints();
    right_elims_6.calculate_waypoints();
    std::vector<Waypoint> right_elims_1_waypoints = right_elims_1.get_waypoints();
    std::vector<Waypoint> right_elims_2_waypoints = right_elims_2.get_waypoints();
    std::vector<Waypoint> right_elims_3_waypoints = right_elims_3.get_waypoints();
    std::vector<Waypoint> right_elims_4_waypoints = right_elims_4.get_waypoints();
    std::vector<Waypoint> right_elims_5_waypoints = right_elims_5.get_waypoints();
    std::vector<Waypoint> right_elims_6_waypoints = right_elims_6.get_waypoints();
    intake.move_voltage(12000);

    ramsete(right_elims_1_waypoints, 3000, false, true);
    wait_until_within({48, -8}, 3.0);
    intake.move_voltage(0);
    wait_until_done();
    move_point({48, -12}, 800, true);
    turn_point({22, -24}, 700);
    intake.move_voltage(12000);
    ramsete(right_elims_2_waypoints, 1500, false, true);
    pros::delay(200);
    hood_piston.set_value(true);
    wait_until_done();
    turn_point({34, -31}, 700);
    ramsete(right_elims_3_waypoints, 2000);
    set_all_sensors(true);
    turn_heading(0, 700);
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    ramsete(right_elims_4_waypoints, 1200, false, true);
    wait_until_within({48, -26}, 5.0);
    lock_piston.set_value(true);
    wait_until_done();
    pros::delay(1300);
    lock_piston.set_value(false);
    set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);

    move_time(-6000, 300);
    loader_piston.set_value(true);
    turn_point({48, -64}, 1000);
    ramsete(right_elims_5_waypoints, 600, false, false, 3.0, 500);

    // for(int i = 0; i < 2; i++) {
    //     move_time(12000, 200);
    //     move_time(0, 200);
    // }

    move_time(-6000, 300);
    loader_piston.set_value(false);
    turn_point({48, -24}, 1000);
    ramsete(right_elims_6_waypoints, 2000);
    lock_piston.set_value(true);

    // move_point({48, -48}, 1000, true);
    // turn_heading(0, 1000);
    // loader_piston.set_value(false);
    // ramsete(right_elims_5_waypoints, 2000);
    // lock_piston.set_value(true);

}

void autonomous() {

    // Pose start = Pose(18, -53, M_PI / 6); // right elims
    Pose start = Pose(6.5, -48, M_PI / 2); // right sawp

    // Pose start = Pose(24, -48, M_PI); // test
    setPose(start);
    initialize_mcl();

    // pros::Task flush_task([]() {
    //     while(true) {
    //         flush_logs();
    //         pros::delay(1000);
    //         // master.rumble(".");
    //     }
    // });

    pros::Task autonomous_task([]() {
        while (true) {
            update_odom();
            update_particles();

            // log_mcl();

            Pose belief = get_pose_estimate();
            belief.theta = getPose().theta;
            setPose(belief);

            // log_mcl();

            resample_particles();

            pros::delay(10);
        }
    });

    // right_elims();
    right_sawp();

}