#include "config.hpp"

miku::Controller master(pros::E_CONTROLLER_MASTER);

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

PIDGains intake_top_gains(20.0, 5.0, 0.0);
PIDGains intake_bottom_gains(20.0, 5.0, 0.0);

miku::Motor intake_top(-1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees, intake_top_lut, intake_top_gains);
miku::Motor intake_bottom(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees, intake_bottom_lut, intake_bottom_gains);

miku::Intake intake(std::make_shared<miku::Motor>(intake_top), std::make_shared<miku::Motor>(intake_bottom));

LookupTable left_drive_lut({
    {-635, -12000}, {-580, -11000}, {-541, -10000}, {-485, -9000}, {-437, -8000}, {-380, -7000},
    {-332, -6000}, {-293, -5500}, {-264, -5000}, {-235, -4500}, {-205, -4000}, {-177, -3500},
    {-152, -3000}, {-115, -2500}, {-82, -2000}, {-52, -1500}, {-30, -1000}, {0, -500},
    {0, 0}, {0, 500}, {30, 1000}, {52, 1500}, {83, 2000}, {110, 2500}, {150, 3000}, {170, 3500},
    {201, 4000}, {230, 4500}, {260, 5000}, {290, 5500}, {328, 6000}, {380, 7000}, {434, 8000},
    {485, 9000}, {539, 10000}, {580, 11000}, {635, 12000}
});

LookupTable right_drive_lut({
    {-617, -12000}, {-580, -11000}, {-537, -10000}, {-485, -9000}, {-437, -8000}, {-380, -7000},
    {-332, -6000}, {-293, -5500}, {-264, -5000}, {-235, -4500}, {-205, -4000}, {-177, -3500}, 
    {-152, -3000}, {-115, -2500}, {-85, -2000}, {-52, -1500}, {-30, -1000}, {0, -500},
    {0, 0}, {0, 500}, {30, 1000}, {52, 1500}, {82, 2000}, {111, 2500}, {150, 3000}, {170, 3500},
    {201, 4000}, {230, 4500}, {263, 5000}, {291, 5500}, {328, 6000}, {380, 7000}, {434, 8000},
    {485, 9000}, {537, 10000}, {575, 11000}, {617, 12000}
});

PIDGains left_drive_gains(0.0, 0.0, 0.0);
PIDGains right_drive_gains(0.0, 0.0, 0.0);

miku::MotorGroup left_motors({-8, -9, -16},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             left_drive_lut,
                             left_drive_gains);
miku::MotorGroup right_motors({15, 2, 3},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             right_drive_lut,
                             right_drive_gains);

pros::Imu imu(4);

miku::Distance north_distance(13, 5.25, -0.75, 0.0f);                // front (0)
miku::Distance south_distance(21, -4.5, -3.5, M_PI);                  // back 
miku::Distance west_distance(20, -5.0, -3.25, -M_PI_2);                 // left (-90deg)
miku::Distance east_distance(5, 5.0, -3.25, M_PI_2);                 // right (90deg)
miku::Distance nw_distance(9, -5.625, -2.625, -M_PI_4);                  // front
miku::Distance ne_distance(11, 5.625, -2.75, M_PI_4);                    // front

ParticleFilter mcl({
    std::make_shared<miku::Distance>(north_distance),
    std::make_shared<miku::Distance>(south_distance),
    std::make_shared<miku::Distance>(west_distance),
    std::make_shared<miku::Distance>(east_distance),
    std::make_shared<miku::Distance>(nw_distance),
    std::make_shared<miku::Distance>(ne_distance)
});

miku::Chassis Miku(
    std::make_shared<miku::MotorGroup>(left_motors),
    std::make_shared<miku::MotorGroup>(right_motors),
    std::make_shared<pros::Imu>(imu),
    std::make_shared<ParticleFilter>(mcl)
);

miku::Pneumatic loader_piston('C');
miku::Pneumatic lock_piston('A');
miku::Pneumatic middle_piston('B');
miku::Pneumatic descore_piston('D');

miku::Optical intake_optical(14);
miku::Optical floor_optical(6);

PIDGains turn_gains(320.0, 0.0, 3000.0);
PIDGains drive_gains(500.0, 0.0, 3000.0);

PatienceExit drive_patience_exit(5, 1, false, 5.0);
PatienceExit turn_patience_exit(5, 1, false, 5.0);