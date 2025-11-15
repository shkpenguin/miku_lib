#include "pros/imu.hpp"

namespace miku {
    class Imu : public pros::Imu {
        using pros::Imu::Imu;

        void calibrate(bool blocking = true);
    };
} // namespace miku