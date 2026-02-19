#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace pros {
    using mutex_t = void*;

    inline uint32_t millis() { return 0; }
    inline void delay(uint32_t) {}

    class Imu {
    public:
        Imu(int port) {}
        double get_rotation() { return 0.0; }
        double get_heading() { return 0.0; }
        void reset() {}
    };

    namespace E_MOTOR_BRAKE {
        enum {
            COAST,
            BRAKE,
            HOLD
        };
    }
    const int E_MOTOR_BRAKE_BRAKE = 1;

    class Motor {
    public:
         Motor(int port) {}
         void move(int voltage) {}
         void move_velocity(int velocity) {}
         double get_position() { return 0.0; }
         double get_velocity() { return 0.0; }
    };

    class Controller {
    public:
        Controller(int id) {}
        void rumble(const char* pattern) {}
    };

    class ADIDigitalOut {
    public:
        ADIDigitalOut(int port) {}
        void set_value(int value) {}
    };
     class ADIDigitalIn {
    public:
        ADIDigitalIn(int port) {}
        int get_value() { return 0; }
    };

    class Distance {
    public:
        Distance(int port) {}
        int get() { return 0; }
    };

     class Optical {
    public:
        Optical(int port) {}
        double get_hue() { return 0.0; }
    };
}
