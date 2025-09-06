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
    {18, -53, 0},
    {22, -45, 40},
    {26, -36, 40},
    {36, -24, 20},
    {46, -9, 0},
    {46, -9, 0}
};

std::vector<ControlPoint> right_elims_cp_2 = {
    {46, -9, 0},
    {46, -9, 20},
    {22, -20, 30},  
    {20, -32, 20},
    {24, -48, 30},
    {42, -48, 30},
    {48, -30, 0},
    {48, -30, 0}
};

BezierPath test_path(test_cp);
std::vector<Waypoint> test_waypoints;

BezierPath right_elims_1(right_elims_cp_1);
BezierPath right_elims_2(right_elims_cp_2);

void setup() {
    turn_heading(30, 10000);
}

void test() {

    pros::delay(500);
    // initialize_mcl({24, -48, 0});
    // test_path.calculate_waypoints();
    // test_waypoints = test_path.get_waypoints();
    // ramsete(test_waypoints, 20000);
    
    // // turn_heading(90, 10000);
    // move_point({0, -24}, 10000);
    move_point({48, -48}, 10000);
}

void right_elims() {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

    right_elims_1.calculate_waypoints();
    right_elims_2.calculate_waypoints();
    std::vector<Waypoint> right_elims_1_waypoints = right_elims_1.get_waypoints();
    std::vector<Waypoint> right_elims_2_waypoints = right_elims_2.get_waypoints();
    intake.move_voltage(12000);

    ramsete(right_elims_1_waypoints, 3000, false, true);
    wait_until_within({46, -8}, 6.0);
    intake.move_voltage(0);
    wait_until_done();
    turn_point({24, -24}, 1000);
    intake.move_voltage(12000);
    ramsete(right_elims_2_waypoints, 6000, false, true);
    pros::delay(3000);
    hood_piston.set_value(true);
    wait_until_done();
    lock_piston.set_value(true);
    pros::delay(1000);
    // move_point({48, -48}, 1000, true);
    // turn_point({48, -72}, 1000);
    // loader_piston.set_value(true);
    // move_point({48, -60}, 1000);

}

void autonomous() {

    // Pose start = Pose(18, -53, 0.523599); // right elims
    Pose start = Pose(24, -48, 90 * (M_PI / 180.0)); // skills
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

    test();
}