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
    
    // set_hood(true);
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void test() {
    
    turn_heading(180, 1000);

    // set_wheel_tracking(false);
    // intake.move_voltage(12000);
    // move_motors(6000, 6000);
    // pros::delay(800);
    // intake.move_voltage(0);
    // move_motors(6000, 12000);
    // pros::delay(1200);
    // stop_motors();
    // set_pose({-48, 36, get_pose().theta});
    // initialize_particles_uniform({-48, 36}, 6.0);
    // set_wheel_tracking(true);
    // turn_heading(90, 1000);

    // move_pose({48, -42, 0}, 2000, {.reverse = true, .cutoff = 6.0});
    // move_point({48, -32}, 1000);

    // ramsete(test_path.get_waypoints(), 4000);
    // wait_until_done();

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