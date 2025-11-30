#include "miku/miku-api.h"
#include "liblvgl/lvgl.h"
#include "gif-pros/gifclass.hpp"

// todo: make new selector

std::vector<lv_obj_t*> auton_list;

lv_obj_t* list_container = nullptr;

lv_obj_t* box = nullptr;
lv_obj_t* label = nullptr;

lv_obj_t* countdown = nullptr;
lv_obj_t* countdownTime = nullptr;

bool prev_left = false;
bool prev_right = false;
bool prev_back = false;

bool new_left = false;
bool new_right = false;
bool new_back = false;
bool new_both = false;

bool both_detected = false;
bool waiting_for_both_release = false;

void update_sensors() {
    new_left = false;
    new_right = false;
    new_back = false;
    new_both = false;

    bool get_left = (left_distance.get_distance() < 10);
    bool get_right = (right_distance.get_distance() < 10);
    bool get_back = (back_distance.get_distance() < 10);

    // --- both-hands gesture ---
    if (get_left && get_right && !both_detected) {
        both_detected = true;
        waiting_for_both_release = true;
    }

    if (waiting_for_both_release && !get_left && !get_right) {
        new_both = true;
        waiting_for_both_release = false;
        both_detected = false;
    }

    if (!(get_left && get_right)) {
        both_detected = false;
    }

    // --- single taps (only if not in both gesture) ---
    if (!waiting_for_both_release) {
        if (!get_left && prev_left) new_left = true;
        if (!get_right && prev_right) new_right = true;
    }

    // --- back sensor logic ---
    if (get_back && !prev_back) {
        new_back = true;
    }

    // save state
    prev_left = get_left;
    prev_right = get_right;
    prev_back = get_back;
}