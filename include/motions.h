#include "mp.h"

void ramsete(std::vector<Waypoint> waypoints, double b = 0.003, double zeta = 0.7, double time_multi = 1.3,
             double cutoff = 6, double kP = 1000, double kD = 450, bool reversed = false, double t_limit = 0);