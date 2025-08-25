#include "api.h"
#include "misc.h"

std::vector<std::pair<double, double>> lookup = {
    {-100.0, -12000}, {-91, -11000}, {-82, -10000}, {-73.7, -9000},
    {-64.4, -8000}, {-55.6, -7000}, {-46.7, -6000}, {-37.3, -5000},
    {-29.1, -4000}, {-20.5, -3000}, {-11.1, -2000}, {-3.9, -1000},
    {0, 0},
    {2.7, 1000}, {10.3, 2000}, {19.6, 3000}, {27.9, 4000},
    {36.1, 5000}, {45.3, 6000}, {54.4, 7000}, {63.2, 8000},
    {72.6, 9000}, {81, 10000}, {90, 11000}, {100, 12000}
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
    if (low == lookup.size() - 1) {
        return lookup[low].second;
    }
    double x0 = lookup[low].first;
    double y0 = lookup[low].second;
    double x1 = lookup[low + 1].first;
    double y1 = lookup[low + 1].second;
    return y0 + (y1 - y0) * (velocity - x0) / (x1 - x0);
}