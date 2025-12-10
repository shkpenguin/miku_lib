#include "miku/miku-api.h"
#include "miku/motions.h"

BezierPath test_path({
});

std::vector<std::reference_wrapper<BezierPath>> test_paths = {std::ref(test_path)};

void test() {
    auto test = new MoveTime(0, 0, 10);
    test->add_event(ConditionalEvent{
        []() { return true; },
        []() { intake.set_anti_jam(true); intake.set({3000, VOLTAGE},
                           {3000, VOLTAGE}); }
    });
    queue_motion(test);
}

