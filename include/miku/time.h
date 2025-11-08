#pragma once

#include "api.h"

class Timer {
    public:
        /**
         * @brief Construct a new Timer
         *
         * @note the timer will start counting down as soon as it is created
         * @note the timer constructor is non-blocking so code after it will be executed immediately
         * @note if the timer is constructed in a global scope, its behavior is undefined. You can
         *       call set() before using the timer if you absolutely need to construct it in a global scope
         *
         * @param time how long to wait in ms
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * @endcode
         */
        Timer(int time);
        Timer() = default;
        /**
         * @brief Get the amount of time the timer was set to
         *
         * @return Time time
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // get the time the timer was set to
         * const Time time = timer.get_time_set(); // time = 1_sec
         * @endcode
         */
        int get_time_set() const;
        /**
         * @brief Get the amount of time left on the timer
         *
         * @return Time time
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time left on the timer
         * const Time time = timer.get_time_left(); // time = 700_msec
         * @endcode
         */
        int get_time_left();
        /**
         * @brief Get the amount of time passed on the timer
         *
         * @return Time time
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time passed on the timer
         * const Time time = timer.getTimePassed(); // time = 300_msec
         * @endcode
         */
        int get_time_passed();
        /**
         * @brief Get whether the timer is done or not
         *
         * @return true the timer is done
         * @return false the timer is not done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // delay for 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.is_done(); // done = false
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.is_done(); // done = true
         * @endcode
         */
        bool is_done();
        /**
         * @brief Get whether the timer is paused or not
         *
         * @return true the timer is paused
         * @return false the timer is not paused
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // pause the timer
         * timer.pause();
         * // check if the timer is paused
         * bool paused = timer.is_paused(); // paused = true
         * // resume the timer
         * timer.resume();
         * // check if the timer is paused
         * paused = timer.is_paused(); // paused = false
         * @endcode
         */
        bool is_paused() const;
        /**
         * @brief Set the amount of time the timer should count down. Resets the timer
         *
         * @param time time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // set the timer to wait for 2 seconds
         * timer.set(2_sec);
         * @endcode
         */
        void set(int time);
        /**
         * @brief reset the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // delay for 500ms
         * pros::delay(500);
         * // reset the timer
         * timer.reset();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void reset();
        /**
         * @brief pause the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // pause the timer
         * timer.pause();
         * // delay for 2000ms
         * pros::delay(2000);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void pause();
        /**
         * @brief resume the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // pause the timer
         * timer.pause();
         * // delay for 500ms
         * pros::delay(500);
         * // resume the timer
         * timer.resume();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void resume();
        /**
         * @brief wait until the timer is done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1_sec);
         * // wait until the timer is done
         * timer.waitUntilDone();
         * std::cout << "done!" << std::endl;
         * @endcode
         */
        void wait_until_done();
    private:
        void update();

        int m_period;
        int m_lastTime = 0;
        int m_timeWaited = 0;
        bool m_paused = false;
};

class Stopwatch {
    public:
        /**
         * @brief Construct a new Stopwatch
         *
         * @b Example
         * @code {.cpp}
         * // create a stopwatch
         * Stopwatch stopwatch;
         * @endcode
         */
        Stopwatch() = default;
        /**
         * @brief Get the time passed since the stopwatch was created or last reset
         *
         * @return Time time
         *
         * @b Example
         * @code {.cpp}
         * // create a stopwatch
         * Stopwatch stopwatch;
         * // delay for 500ms
         * pros::delay(500);
         * // get the time passed
         * const Time time = stopwatch.get_time_passed(); // time = 500_msec
         * @endcode
         */
        int get_time_passed();
        /**
         * @brief reset the stopwatch
         *
         * @b Example
         * @code {.cpp}
         * // create a stopwatch
         * Stopwatch stopwatch;
         * // delay for 500ms
         * pros::delay(500);
         * // reset the stopwatch
         * stopwatch.reset();
         * // delay for another 500ms
         * pros::delay(500);
         * // get the time passed
         * const Time time = stopwatch.get_time_passed(); // time = 500_msec
         * @endcode
         */
        void reset();
    private:
        int m_startTime = pros::millis();
};