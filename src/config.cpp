#include "config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-11, -12, 13});
pros::MotorGroup right_motors({20, 19, -18});

pros::MotorGroup intake({-1, 10});

pros::Motor left_front(-12);
pros::Motor left_middle(-12);
pros::Motor left_back(13);
pros::Motor right_front(20);
pros::Motor right_middle(19);
pros::Motor right_back(-18);

pros::Motor top_intake(-1);
pros::Motor bottom_intake(10);

pros::Imu imu(3);

pros::adi::DigitalOut hood_piston('C');
pros::adi::DigitalOut lock_piston('A');
pros::adi::DigitalOut loader_piston('E');
pros::adi::DigitalOut descore_piston('D');

Gains drive_gains(500.0, 0.0, 3000.0);
Gains turn_gains(340.0, 10.0, 4100.0);

ExitCondition drive_small_exit(1.0, 200);
ExitCondition drive_large_exit(5.0, 1000);

ExitCondition turn_small_exit(1.0, 100);
ExitCondition turn_large_exit(3.0, 500);