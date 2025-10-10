#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath test_path = {
    {6, -48, 90},
    {6, -48, 100},
    {24, -48, 80},
    {42, -48, 15},
    {48, -48, 0},
    {48, -48, 0}
};

std::vector<std::reference_wrapper<BezierPath>> test_paths = {
    std::ref(test_path)
};

void pre_test() {
    
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void test() {
    
    // intake.move_voltage(12000);
    // move_motors(6000, 6000);
    // ExitCondition pitch_exit(0.5, 500);
    // pitch_exit.reset();
    // pros::delay(1000);
    // Timer timer(1000);
    // while(!pitch_exit.getExit() && !timer.isDone()) {
    //     pitch_exit.update(imu.get_pitch());
    //     pros::delay(10);
    // }
    // move_motors(0, 0);

    // move_pose({48, -42, 0}, 2000, {.reverse = true, .cutoff = 6.0});
    // move_point({48, -32}, 1000);

    ramsete(test_path.get_waypoints(), 4000);
    wait_until_done();

}

void move_goal() {
    int quadrant = find_quadrant(get_pose());
    if(quadrant == 1) {
        move_pose({48, 42, 180}, 2000);
        move_point({48, 32}, 1000);
    }
    if(quadrant == 2) {
        move_pose({-48, 42, 180}, 2000);
        move_point({-48, 32}, 1000);
    }
    if(quadrant == 3) {
        move_pose({-48, -42, 180}, 2000);
        move_point({-48, -32}, 1000);
    }
    if(quadrant == 4) {
        move_pose({48, -42, 0}, 2000);
        move_point({48, -32}, 1000);
    }
}