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

pros::adi::DigitalOut hood_piston('C');
pros::adi::DigitalOut lock_piston('A');
pros::adi::DigitalOut loader_piston('H');
pros::adi::DigitalOut descore_piston('D');

Gains drive_gains(500.0, 0.0, 1000.0);
Gains turn_gains(80.0, 0.0, 400.0);

ExitCondition drive_small_exit(1.0, 400);  
ExitCondition drive_large_exit(3.0, 800); 

ExitCondition turn_small_exit(1.0, 100);
ExitCondition turn_large_exit(3.0, 500);