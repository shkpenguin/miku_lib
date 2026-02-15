#include "config.hpp"

miku::Controller master(pros::E_CONTROLLER_MASTER);

LookupTable intake_bottom_lut({
    
});

LookupTable intake_middle_lut({
    
});

LookupTable intake_top_lut({

});

PIDGains intake_top_gains(20.0, 5.0, 0.0);
PIDGains intake_middle_gains(20.0, 5.0, 0.0);
PIDGains intake_bottom_gains(20.0, 5.0, 0.0);

miku::Motor intake_top(-8, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees, intake_top_lut, intake_top_gains);
miku::Motor intake_middle(-18, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees, intake_middle_lut, intake_middle_gains);
miku::Motor intake_bottom(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees, intake_bottom_lut, intake_bottom_gains);

miku::Intake intake(std::make_shared<miku::Motor>(intake_top), std::make_shared<miku::Motor>(intake_middle), std::make_shared<miku::Motor>(intake_bottom));

LookupTable left_drive_lut({
    {-680, -12000},
    {-630, -11000},
    {-570, -10000},
    {-510, -9000},
    {-480, -8500},
    {-450, -8000},
    {-420, -7500},
    {-390, -7000},
    {-360, -6500},
    {-338, -6000},
    {-300, -5500},
    {-265, -5000},
    {-234, -4500},
    {-210, -4000},
    {-182, -3500},
    {-152, -3000},
    {-116, -2500},
    {-86, -2000},
    {-56, -1500},
    {-32, -1000},
    {0, -500},
    {0, 0},
    {0, 500},
    {28, 1000},
    {54, 1500},
    {82, 2000},
    {106, 2500},
    {148, 3000},
    {179, 3500},
    {208, 4000},
    {233, 4500},
    {260, 5000},
    {300, 5500},
    {335, 6000},
    {360, 6500},
    {390, 7000},
    {420, 7500},
    {450, 8000},
    {480, 8500},
    {510, 9000},
    {570, 10000},
    {630, 11000},
    {680, 12000}
});

LookupTable right_drive_lut({
    {-680, -12000},
    {-630, -11000},
    {-570, -10000},
    {-510, -9000},
    {-480, -8500},
    {-450, -8000},
    {-420, -7500},
    {-390, -7000},
    {-360, -6500},
    {-338, -6000},
    {-300, -5500},
    {-265, -5000},
    {-234, -4500},
    {-210, -4000},
    {-182, -3500},
    {-152, -3000},
    {-116, -2500},
    {-86, -2000},
    {-56, -1500},
    {-32, -1000},
    {0, -500},
    {0, 0},
    {0, 500},
    {28, 1000},
    {54, 1500},
    {82, 2000},
    {106, 2500},
    {148, 3000},
    {179, 3500},
    {208, 4000},
    {233, 4500},
    {260, 5000},
    {300, 5500},
    {335, 6000},
    {360, 6500},
    {390, 7000},
    {420, 7500},
    {450, 8000},
    {480, 8500},
    {510, 9000},
    {570, 10000},
    {630, 11000},
    {680, 12000}
});

PIDGains left_drive_gains(20.0, 5.0, 40.0);
PIDGains right_drive_gains(20.0, 5.0, 40.0);

miku::MotorGroup left_motors({-11, -12, -13},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             left_drive_lut,
                             left_drive_gains);
miku::MotorGroup right_motors({15, 16, 17},
                             pros::v5::MotorGears::blue,
                             pros::v5::MotorUnits::degrees,
                             right_drive_lut,
                             right_drive_gains);

pros::Imu imu(4);

miku::Distance north_distance(5, -3.4, 6, 0);
miku::Distance south_distance(7, 2.5, -2.5, 180);
miku::Distance west_distance(9, -4, 3.75, -90);
miku::Distance east_distance(10, 4, 3.75, 90);

// legacy / convenience names used elsewhere in the codebase —
// reference the concrete sensor objects above so we don't create
// duplicate pros::Distance instances for the same ports.
miku::Distance &front_distance = north_distance;
miku::Distance &back_distance  = south_distance;
miku::Distance &left_distance  = west_distance;
miku::Distance &right_distance = east_distance;
// miku::Distance nw_distance(9, -5.625, -2.625, -45);
// miku::Distance ne_distance(11, 5.625, -2.75, 45);

ParticleFilter mcl({
    std::make_shared<miku::Distance>(north_distance),
    std::make_shared<miku::Distance>(south_distance),
    std::make_shared<miku::Distance>(west_distance),
    std::make_shared<miku::Distance>(east_distance),
    // std::make_shared<miku::Distance>(nw_distance),
    // std::make_shared<miku::Distance>(ne_distance)
});

miku::Chassis Miku(
    std::make_shared<miku::MotorGroup>(left_motors),
    std::make_shared<miku::MotorGroup>(right_motors),
    std::make_shared<pros::Imu>(imu),
    std::make_shared<ParticleFilter>(mcl)
);

miku::Pneumatic middle_piston('E');
miku::Pneumatic loader_piston('F');
miku::Pneumatic lock_piston('G');
miku::Pneumatic descore_piston('H');

// miku::Optical intake_optical(14);
// miku::Optical floor_optical(6);

PIDGains turn_gains(340.0, 0.0, 3100.0);
PIDGains drive_gains(650.0, 0.0, 5000.0);

PatienceExit drive_quick_exit(2, 1, false, 5.0);
PatienceExit drive_slow_exit(5, 0.5, false, 5.0);
PatienceExit turn_patience_exit(2, 1, false, 5.0);