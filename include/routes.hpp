#pragma once
#include <string>
#include <functional>
#include "miku/mp.hpp"
#include "miku/geometry.hpp"

struct Route {
    std::string name;
    Pose start_pose;
    std::function<void()> queue;
    std::vector<std::reference_wrapper<BezierPath>> paths;
    Route(std::string name, Pose start_pose, std::function<void()> queue,  std::vector<std::reference_wrapper<BezierPath>> paths)
        : name(name), start_pose(start_pose), queue(queue), paths(paths) {}
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