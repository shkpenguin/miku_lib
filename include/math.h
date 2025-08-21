#pragma once

#include <cmath>

double clamp(double value, double min, double max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

double clamp_field(double value) {
    return clamp(value, -72.0, 72.0);
}