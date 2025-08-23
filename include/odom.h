#pragma once

struct Pose {
    double x; 
    double y;
    double theta; 

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0) 
        : x(x), y(y), theta(theta) {}
};

struct Particle {
    Pose pose; 
    double weight; 
};

Pose getPose(bool standard = false);
Pose getSpeed();
Pose setPose(Pose new_pose);

void track_odom();
void init_odom(Pose inital_pose);