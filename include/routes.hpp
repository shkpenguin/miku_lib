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

void skills();

void sawp();

void right_rush();

void left_rush();

void skills_mid_control();