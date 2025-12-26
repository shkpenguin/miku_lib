#pragma once
#include <string>
#include <functional>
#include "miku/mp.hpp"
#include "miku/geometry.hpp"

struct Route {
    std::string name;
    Pose start_pose;
    std::function<void()> queue;
    Route(std::string name, Pose start_pose, std::function<void()> queue)
        : name(name), start_pose(start_pose), queue(queue) {}
    Route() = default;
};

void test();
extern std::vector<std::reference_wrapper<BezierPath>> test_paths;

void skills();
extern std::vector<std::reference_wrapper<BezierPath>> skills_paths;

void sawp();
extern std::vector<std::reference_wrapper<BezierPath>> sawp_paths;

void right_rush();
extern std::vector<std::reference_wrapper<BezierPath>> rush_paths;

void skills_mid_control();
extern std::vector<std::reference_wrapper<BezierPath>> skills_mid_control_paths;