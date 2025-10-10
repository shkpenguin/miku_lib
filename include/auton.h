#pragma once

#include "mp.h"
#include "odom.h"

class Auton {

    public:
    std::string name;
    std::function<void()> pre_auton;
    std::function<void()> auton;
    
    Pose start_pose;
    std::vector<std::reference_wrapper<BezierPath>> paths;

    Auton(std::string name, std::function<void()> pre_auton, std::function<void()> auton, Pose start_pose, std::vector<std::reference_wrapper<BezierPath>> paths)
        : name(name), pre_auton(pre_auton), auton(auton), start_pose(start_pose), paths(paths) {}
    Auton() = default;

};

extern std::vector<Auton> autons;
extern int selected_index;

void display_selector();
extern pros::Task* autonomous_task;

extern std::vector<std::reference_wrapper<BezierPath>> test_paths;
extern std::vector<std::reference_wrapper<BezierPath>> right_sawp_paths;
extern std::vector<std::reference_wrapper<BezierPath>> right_9ball_paths;
extern std::vector<std::reference_wrapper<BezierPath>> skills_paths;

void pre_test();
void test();

void pre_right_sawp();
void right_sawp();

void pre_right_9ball();
void right_9ball();

void pre_skills();
void skills();