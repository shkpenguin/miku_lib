#pragma once

#include <vector>

class LookupTable {
    private:
        std::vector<std::pair<double, double>> table;
    public:
        LookupTable() {};
        LookupTable(std::vector<std::pair<double, double>> table) : table(table) {};
        LookupTable(std::initializer_list<std::pair<double, double>> table) : table(table) {};
        double get_value(double key);
};

void tune_lut_drive();
void tune_lut_intake();