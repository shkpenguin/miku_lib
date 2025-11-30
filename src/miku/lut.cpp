#include "lut.h"
#include "config.h"
#include <numeric>

float LookupTable::get_value(float key) {
    // use binary search to find least voltage below desired velocity
    int low = 0;
    int high = table.size() - 1;
    while (low < high) {
        int mid = (low + high + 1) / 2;
        if (table[mid].first <= key) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    // use linear interpolation
    if (low == table.size() - 1) {
        return table[low].second;
    }
    float x0 = table[low].first;
    float y0 = table[low].second;
    float x1 = table[low + 1].first;
    float y1 = table[low + 1].second;
    return y0 + (y1 - y0) * (key - x0) / (x1 - x0);
}