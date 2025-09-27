#include "config.h"
#include "liblvgl/lvgl.h"
#include "util.h"
#include "autons.h"
#include "timer.h"
#include "gif-pros/gifclass.hpp"

std::vector<Auton> test_autons = {
    Auton("Test 1", nullptr, nullptr, Pose(0,0,0), {}),
    Auton("Test 2", nullptr, nullptr, Pose(0,0,0), {}),
    Auton("Test 3", nullptr, nullptr, Pose(0,0,0), {}),
};

int selected_index = 0;
Auton selected_auton;

pros::Distance left_sensor(2);
pros::Distance right_sensor(9);
pros::Distance back_sensor(10);

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

void make_selection(std::vector<lv_obj_t*>& list) {

    for(int i = 0; i < list.size(); i++) {
        lv_obj_t* btn = list[i];
        if (i == selected_index) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN); // red highlight
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(i == auton_list.size() - 1 ? 0xAAAAAA : 0x550000), LV_PART_MAIN); // Normal or grey for back option
        }
    }

    selected_auton = test_autons[selected_index];
    selected_auton.pre_auton();
    for(auto& path : selected_auton.paths) {
        path->calculate_waypoints();
    }

}

void highlight_selected(std::vector<lv_obj_t*>& list) {
    for (int i = 0; i < list.size(); i++) {
        lv_obj_t* btn = list[i];
        if (i == selected_index) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x666666), LV_PART_MAIN); // gray highlight
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(i == list.size() - 1 ? 0xAAAAAA : 0x550000), LV_PART_MAIN); // Normal or grey for back option
        }
    }
}

void update_sensors() {
    new_left = false;
    new_right = false;
    new_back = false;
    new_both = false;

    bool get_left = (left_sensor.get_distance() < 10);
    bool get_right = (right_sensor.get_distance() < 10);
    bool get_back = (back_sensor.get_distance() < 10);

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

static void auton_btn_event_cb(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);

    // Find which button in auton_list this is
    for (int i = 0; i < auton_list.size(); i++) {
        if (auton_list[i] == btn) {
            selected_index = i;
            break;
        }
    }

    // Update button highlights
    make_selection(auton_list);
}

void create_auton_list() {
    for(int i = 0; i < test_autons.size(); i++) {
        Auton auton = test_autons[i];
        lv_obj_t* btn = lv_btn_create(list_container); // Attach to list_container
        lv_obj_set_size(btn, 480, 40);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x550000), LV_PART_MAIN);
        
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, auton.name.c_str());
        lv_obj_center(lbl);

        auton_list.push_back(btn);

        lv_obj_add_event_cb(btn, auton_btn_event_cb, LV_EVENT_CLICKED, nullptr);
    }

    highlight_selected(auton_list);
}

void display_selector() {

    int current_display = 0;

    list_container = lv_list_create(lv_scr_act());
    lv_obj_set_size(list_container, 480, 240 - 50);
    lv_obj_set_pos(list_container, 0, 50);
    lv_obj_set_flex_flow(list_container, LV_FLEX_FLOW_COLUMN);

    while (true) {
        selected_auton = test_autons[selected_index];

        update_sensors();
        if(new_back && !imu.is_calibrating()) {
            imu.reset();
            while(imu.is_calibrating()) pros::delay(50);
        }

        if(new_both) { // confirm selection
            make_selection(auton_list);
        }
        if (new_left) {
            selected_index--;
            if (selected_index < 0) selected_index = auton_list.size() - 1;
            highlight_selected(auton_list);
        } else if (new_right) {
            selected_index++;
            if (selected_index >= auton_list.size()) selected_index = 0;
            highlight_selected(auton_list);
        }

        pros::delay(100);
    }
}