#pragma once
#include <string>
#include <functional>
#include "miku/mp.h"

struct Route {
    std::string name;
    std::function<void()> queue;
    std::vector<std::reference_wrapper<BezierPath>> paths;
    Route(std::string name, std::function<void()> queue, std::vector<std::reference_wrapper<BezierPath>> paths)
        : name(name), queue(queue), paths(paths) {}
    Route() = default;
};

void test();
extern std::vector<std::reference_wrapper<BezierPath>> test_paths;

void skills();
extern std::vector<std::reference_wrapper<BezierPath>> skills_paths;