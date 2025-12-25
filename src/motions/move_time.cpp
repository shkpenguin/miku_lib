#include "miku/motions.hpp"

MoveTime::MoveTime(float left_speed, float right_speed, float duration)
        : left_speed(left_speed), right_speed(right_speed), duration(duration) {}

void MoveTime::start() {
    done = false;
    start_time = pros::millis();
    timer.set(duration);
    timer.reset();
    Miku.move_voltage(left_speed, right_speed);
}

void MoveTime::update() {
    if (timer.is_done()) {
        done = true;
        Miku.stop();
        return;
    }
}

void MoveTime::stop() {
    done = true;
    Miku.stop();
}

bool MoveTime::is_done() {
    return done;
}