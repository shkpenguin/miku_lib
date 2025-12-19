#include "miku/miku-api.h"
#include "miku/motions.h"

BezierPath test_path({
});

std::vector<std::reference_wrapper<BezierPath>> test_paths = {std::ref(test_path)};

void test() {
    // queue_motion(new MovePose({24, 0}, 90, 10000, {.reverse = true}));
}