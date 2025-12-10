#include "macro.h"
#include "miku/miku-api.h"

void descore_align() {
    Miku.set_position({0, 0});
    if(Miku.get_heading().degrees().norm() > 180) {
        queue_motion(new MovePoint({12, -12}, 1000));
        queue_motion(new TurnHeading(180, 1000));
        queue_motion(new MovePoint({12, 12}, 1000, {.reverse = true}));
    } else {
        queue_motion(new MovePoint({12, 12}));
        queue_motion(new TurnHeading(0, 1000));
        queue_motion(new MovePoint({-12, 12}, 1000, {.reverse = true}));
    }
}