#include "config.h"

LookupTable left_motors_lut({
    {}
});

LookupTable right_motors_lut({
    {}
});

PIDGains left_motors_gains(0.0, 0.0, 0.0);
PIDGains right_motors_gains(0.0, 0.0, 0.0);

miku::Controller master(pros::E_CONTROLLER_MASTER);

miku::MotorGroup left_motors({-8, -9, -16},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             left_motors_lut,
                             left_motors_gains);
miku::MotorGroup right_motors({15, 2, 3},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             right_motors_lut,
                             right_motors_gains);
miku::Chassis Miku(std::make_shared<miku::MotorGroup>(left_motors), std::make_shared<miku::MotorGroup>(right_motors));

LookupTable intake_bottom_lut({
    {-670, -12000}, {-600, -11000}, {-540, -10000}, {-500, -9000}, {-435, -8000}, {-380, -7000}, 
    {-323, -6000}, {-265, -5000}, {-205, -4000}, {-150, -3000}, {-85, -2000}, {-35, -1000}, 
    {0, 0}, {25, 1000}, {80, 2000}, {147, 3000}, {200, 4000}, {260, 5000}, {320, 6000},
    {380, 7000}, {435, 8000}, {500, 9000}, {555, 10000}, {600, 11000}, {670, 12000}
});

LookupTable intake_top_lut({
    {-600, -12000}, {-565, -11000}, {-520, -10000}, {-460, -9000}, {-400, -8000}, {-345, -7000}, 
    {-288, -6000}, {-230, -5000}, {-175, -4000}, {-115, -3000}, {-50, -2000}, {0, -1000}, {0, 0},
    {0, 1000}, {50, 2000}, {115, 3000}, {175, 4000}, {230, 5000}, {290, 6000}, {350, 7000},
    {400, 8000}, {460, 9000}, {520, 10000}, {560, 11000}, {600, 12000}
});

PIDGains intake_top_gains(0.0, 0.0, 0.0);
PIDGains intake_bottom_gains(20.0, 5.0, 0.0);

miku::Motor intake_top(-1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees, intake_top_lut, intake_top_gains);
miku::Motor intake_bottom(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees, intake_bottom_lut, intake_bottom_gains);

pros::Imu imu(3);

miku::Pneumatic loader_piston('C');
miku::Pneumatic lock_piston('A');
miku::Pneumatic middle_piston('B');
miku::Pneumatic descore_piston('D');

miku::Optical optical(16);

PIDGains turn_gains(4.0, 0.0, 20.0);
PIDGains drive_gains(500.0, 0.0, 1000.0);

miku::Distance back_distance(9, 3.0, -5.5, Orientation::BACK);
miku::Distance left_distance(4, -5.2, 0, Orientation::LEFT);
miku::Distance right_distance(8, 5.2, 0, Orientation::RIGHT);
miku::Distance front_distance(6, -6.5, 8.25, Orientation::FRONT);

PatienceExit drive_patience_exit(0.2, 5);
PatienceExit turn_patience_exit(1, 5);

RangeExit turn_small_exit(1.0, 100);
RangeExit turn_large_exit(3.0, 500);