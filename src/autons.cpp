#include "mp.h"
#include "motions.h"
#include "main.h"
#include <vector>

void test() {
    // turn_heading(90, false, 10000);
    move_point({24, -24}, false, 10000);
}

void autonomous() {
    test();
}