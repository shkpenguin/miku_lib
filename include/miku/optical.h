#include "api.h"

struct Color {
    double min_hue;
    double max_hue;
};

#define TILE Color{40.0, 50.0} // Color of VEX field tiles
#define RED Color{0.0, 20.0} // Bright red VEX field element color
#define BLUE Color{180.0, 220.0} // Bright blue VEX field element color

namespace miku {
    class Optical : public pros::Optical {
        public:
        Optical(const std::uint8_t port) : pros::Optical(port) {}
        bool get_color(const Color& color) {
            double hue = get_hue();
            return (hue >= color.min_hue && hue <= color.max_hue);
        }
        void initialize() {
            this->set_integration_time(5);
            this->set_led_pwm(100);
        }
    };
}