#include "api.h"

std::vector<std::pair<double, double>> lookup = {
    {0, 1.5},
    {10, 2.0},
    {20, 2.5},
    {30, 3.0},
    {40, 3.5},
    {50, 4.0},
    {60, 4.5},
    {70, 5.0},
    {80, 5.5},
    {90, 6.0},
    {100, 6.5}
};

double voltage_lookup(double velocity) {
    // use binary search to find least voltage below desired velocity
    int low = 0;
    int high = lookup.size() - 1;
    while (low < high) {
        int mid = (low + high + 1) / 2;
        if (lookup[mid].first <= velocity) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    // use linear interpolation
    double x0 = lookup[low].first;
    double y0 = lookup[low].second;
    double x1 = lookup[low + 1].first;
    double y1 = lookup[low + 1].second;
    return y0 + (y1 - y0) * (velocity - x0) / (x1 - x0);
}