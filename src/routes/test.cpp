#include "motions.h"
#include "auton.h"
#include "mp.h"

BezierPath test_path = {
    {0, -48, 0},
    {0, -48, 20},
    {24, -48, 20},
    {48, -48, 20},
    {48, -48, 0}
};

std::vector<std::reference_wrapper<BezierPath>> test_paths = {
    std::ref(test_path)
};

void pre_test() {
    
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);

}

void test() {
    // test_path.calculate_waypoints();
    // ramsete(test_path.get_waypoints(), 10000);
    // master.rumble("...");

    turn_point({24, -24}, 2000);
}