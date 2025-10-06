#pragma once

#include <vector>

class LookupTable {
    private:
        std::vector<std::pair<double, double>> table;
    public:
        LookupTable(std::vector<std::pair<double, double>> table) : table(table) {};
        double get_voltage(double velocity);
};

void tune_lut_drive();
void tune_lut_intake();

extern LookupTable drive_lut;
extern LookupTable intake_lut;