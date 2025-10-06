#pragma once

#include "mp.h"
#include "odom.h"

class Auton {

    public:
    std::string name;
    std::function<void()> pre_auton;
    std::function<void()> auton;
    
    Pose start_pose;
    std::vector<std::shared_ptr<BezierPath>> paths;

    Auton(std::string name, std::function<void()> pre_auton, std::function<void()> auton, Pose start_pose, std::vector<std::shared_ptr<BezierPath>> paths)
        : name(name), pre_auton(pre_auton), auton(auton), start_pose(start_pose), paths(paths) {}
    Auton() = default;

};

extern std::vector<Auton> autons;
extern int selected_index;

void set_tracking(bool enabled);

bool get_tracking();

void display_selector();
extern pros::Task* autonomous_task;