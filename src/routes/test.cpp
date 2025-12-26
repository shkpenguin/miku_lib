#include "miku/miku-api.hpp"
#include "miku/motions.hpp"

void test() {
    wait(100)
        .event(start([]() { intake.set(4000, 12000); loader_piston.set_value(true); }))
        .queue();
    move_time(8000, 8000, 1500)
        .seq({
            ConditionalEvent(
                []() { return floor_optical.get_color(BLUE); },
                []() { loader_piston.set_value(false); }),
            await([]() { return floor_optical.get_color(TILE); }),
            await([]() { return floor_optical.get_color(BLUE); })
        })
        .end_seq([]() { return floor_optical.get_color(TILE); })
        .queue();
}