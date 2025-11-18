#include "miku/miku-api.h"
#include "miku/motions.h"

void test() {
    queue_motion(new MovePoint({24, -24}, 10000));
}