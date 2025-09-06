#include "config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-12, -13, -14});
pros::MotorGroup right_motors({19, 18, 17});

pros::MotorGroup intake({11, -20});

pros::Motor left_front(-12);
pros::Motor left_middle(-13);
pros::Motor left_back(-14);
pros::Motor right_front(19);
pros::Motor right_middle(18);
pros::Motor right_back(17);

pros::Motor top_intake(11);
pros::Motor bottom_intake(-20);

pros::Imu imu(21);

pros::adi::DigitalOut hood_piston('A');
pros::adi::DigitalOut lock_piston('B');
pros::adi::DigitalOut loader_piston('C');
pros::adi::DigitalOut descore_piston('D');

Gains drive_gains(200.0, 0.0, 0.0);
Gains turn_gains(450.0, 50.0, 3950.0);
Gains angular_gains(7.8, 0.8, 69.0);

ExitCondition drive_small_exit(1.0, 200);
ExitCondition drive_large_exit(5.0, 1000);

ExitCondition turn_small_exit(1.0, 100);
ExitCondition turn_large_exit(3.0, 500);