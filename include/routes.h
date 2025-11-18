#pragma once
#include <string>
#include <functional>

struct Route {
    std::string name;
    std::function<void()> queue;
    Route(std::string name, std::function<void()> queue)
        : name(name), queue(queue) {}
    Route() {}
};

void test();