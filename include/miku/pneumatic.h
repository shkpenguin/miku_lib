#include "pros/adi.hpp"

namespace miku {
class Pneumatic : public pros::adi::DigitalOut {
bool state;
public:
    Pneumatic(uint8_t port) : pros::adi::DigitalOut(port), state(false) {}
    void toggle() {
        state = !state;
        set_value(state);
    }
    bool get_value() const {
        return state;
    }
};
} // namespace miku