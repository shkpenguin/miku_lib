#include "main.h"
#include "auton.h"
#include "miku-api.h"
#include <vector>

pros::Task* autonomous_task = nullptr;
pros::Task* intake_task = nullptr;
pros::Task* controller_task = nullptr;

std::vector<Auton> autons;

void init_autons() {
    autons = {
        Auton("Test", pre_test, test, Pose(0, 0, 0, false), test_paths),
        Auton("Right Sawp", pre_right_sawp, right_sawp, Pose(6.5, -48, 90, false), right_sawp_paths),
        Auton("Right 9 Ball", pre_right_9ball, right_9ball, Pose(8, -48, 30, false), right_9ball_paths),
        Auton("Skills", pre_skills, skills, Pose(6, -48, 90, false), skills_paths)
    };
}

void initialize() {

    left_motors.tare_position_all();
    right_motors.tare_position_all();
    intake.tare_position_all();

    master.set_text(0, 0, "IMU Calibrating");

    static Gif gif("/usd/jiachenma.gif", lv_scr_act());

    imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(10);
	}

    init_autons();

    master.set_text(0, 0, "              ");

    intake_task = new pros::Task(intake_control);

    selected_index = 3;

    if (autons.empty()) return;
    // display_selector();

    Auton& selected_auton = autons[selected_index];
    selected_auton.pre_auton();
    for(auto& path : selected_auton.paths) {
        path.get().calculate_waypoints();
    }

    initialize_pose(selected_auton.start_pose);

    controller_task = new pros::Task(display_controller);

}

void autonomous() { 
    #if LOGGING_ENABLED
    file.open("log.txt");
    #endif
    // intake_task = new pros::Task(intake_control);

    autonomous_task = new pros::Task([]() {

        uint32_t prev_time = pros::millis();

        while (true) {

            update_odom();
            update_particles();

            Pose belief = get_pose_estimate();
            belief.theta = get_pose().theta;
            set_pose(belief);

            resample_particles();

            pros::Task::delay_until(&prev_time, DELTA_TIME);

        }

    });

    #if LOGGING_ENABLED
    pros::Task log_task = pros::Task([]() {
        while (true) {
            flush_logs();
            pros::delay(1000);
        }
    });
    #endif

    Auton& selected_auton = autons[selected_index];
    selected_auton.auton(); 

}

enum class DriveMode {
    TANK = 0,
    ARCADE = 1,
    FUNNY_TANK = 2
};

int curve(int pos) {
    if(fabs(pos) <= 5) return 0;
    return (pos * pos * pos * 0.00337827447) + (pos * 40);
}

void tank(int left, int right) {
    left_motors.move(left);
    right_motors.move(right);
}

void funny_tank(int left_x, int left_y, int right_x, int right_y) {
    if(fabs(left_x) > 50 && fabs(right_x) > 50) {
        int sign = (left_x > 0 || right_x < 0) ? 1 : -1;
        left_x = (fabs(left_x) - 50) * 127 / 77;
        right_x = (fabs(right_x) - 50) * 127 / 77;
        double speed = (fabs(left_x) + fabs(right_x)) / 2.0 * sign;
        left_motors.move(speed);
        right_motors.move(speed);
    } else {
        left_motors.move(left_y);
        right_motors.move(right_y);
    }
}

void arcade(int throttle, int turn) {
    int left = throttle + turn;
    int right = throttle - turn;

    left_motors.move(left);
    right_motors.move(right);
}

DriveMode driveMode = DriveMode::FUNNY_TANK;

void opcontrol() {

    // rumble_timer.resume();
    // if(autonomous_task != nullptr) autonomous_task->remove();
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

    while (true) {

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            if(!get_intake_tbh()) set_intake_velocity(200);
            else set_intake_tbh(false);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            driveMode = (driveMode == DriveMode::TANK) ? DriveMode::ARCADE : DriveMode::TANK;
            master.rumble("."); // Short vibration to indicate mode change
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            test();
        }

        if(driveMode == DriveMode::TANK) {
            int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            tank(left, right);
        } else if(driveMode == DriveMode::ARCADE) {
            int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            arcade(throttle, turn);
        } else if(driveMode == DriveMode::FUNNY_TANK) {
            int left_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            funny_tank(left_x, left_y, right_x, right_y);
        }

        bool shift1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool shift2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if(!shift1 && !shift2) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !get_intake_tbh()) {
                intake.move_voltage(12000);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                lock = !lock;
                lock_piston.set_value(lock);
            }
        }

        else if(shift1) {
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move_voltage(-12000);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                descore = !descore;
                descore_piston.set_value(descore);
            }
        }

        if(shift2 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            loading = !loading;
            loader_piston.set_value(loading);
        }

        bool hood_toggle_pressed =
        (shift2 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) ||
        (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

        if((!loading || !hood_up) && 
           (hood_toggle_pressed)) {
            hood_up = !hood_up;
            hood_piston.set_value(hood_up);
        }

        if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !get_intake_tbh()) {
            intake.move_voltage(0);
        }

        pros::delay(10);
    }
}