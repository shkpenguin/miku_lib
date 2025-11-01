#include "config.h"
#include "pid.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

miku::MotorGroup left_motors({-11, -12, 13});
miku::MotorGroup right_motors({20, 19, -18});

miku::MotorGroup intake({-1, 10});

miku::Motor left_front(-12);
miku::Motor left_middle(-12);
miku::Motor left_back(13);
miku::Motor right_front(20);
miku::Motor right_middle(19);
miku::Motor right_back(-18);

miku::Motor top_intake(-1);
miku::Motor bottom_intake(10);

pros::Imu imu(3);

miku::Pneumatic hood_piston('C');
miku::Pneumatic lock_piston('A');
miku::Pneumatic loader_piston('H');
miku::Pneumatic descore_piston('B');

miku::Optical optical(16);

Gains turn_gains(4.0, 0.0, 20.0);
Gains drive_gains(500.0, 0.0, 1000.0);

miku::Distance back_distance(9, 3.0, -5.5, Orientation::BACK);
miku::Distance left_distance(4, -5.2, 0, Orientation::LEFT);
miku::Distance right_distance(8, 5.2, 0, Orientation::RIGHT);
miku::Distance front_distance(6, -6.5, 8.25, Orientation::FRONT);

RangeExit drive_small_exit(1.0, 400);  
RangeExit drive_large_exit(3.0, 800); 

RangeExit turn_small_exit(1.0, 100);
RangeExit turn_large_exit(3.0, 500);