#include "timer.h"

Timer::Timer(int time)
    : m_period(time) {
    m_lastTime = pros::millis();
}

void Timer::update() {
    const int time = pros::millis(); // get current time from RTOS
    if (!m_paused) m_timeWaited += time - m_lastTime; // dont update if paused
    m_lastTime = time; // update last time
}

int Timer::getTimeSet() {
    this->update();
    return m_period;
}

int Timer::getTimeLeft() {
    this->update();
    const int delta = m_period - m_timeWaited; // calculate how much time is left
    return (delta > 0) ? delta : 0; // return 0 if timer is done
}

int Timer::getTimePassed() {
    this->update();
    return m_timeWaited;
}

bool Timer::isDone() {
    this->update();
    int delta = m_period - m_timeWaited; // calculate how much time is left
    return delta <= 0;
}

bool Timer::isPaused() {
    const int time = pros::millis(); // get time from RTOS
    if (!m_paused) m_timeWaited += time - m_lastTime; // dont update if paused
    return m_paused;
}

void Timer::set(int time) {
    m_period = time; // set how long the timer should run
    this->reset();
}

void Timer::reset() {
    m_timeWaited = 0;
    m_lastTime = pros::millis();
}

void Timer::pause() {
    if (!m_paused) {
        const int now = pros::millis();  // get current time first
        m_timeWaited += now - m_lastTime; // accumulate elapsed time
        m_paused = true;                  // then mark paused
    }
}

void Timer::resume() {
    if (m_paused) m_lastTime = pros::millis();
    m_paused = false;
}

void Timer::waitUntilDone() {
    do pros::delay(5);
    while (!this->isDone());
}