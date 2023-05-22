#pragma once

#include "src/common.hpp"

string EditmodeStr[5]{"Trans", "ScalingUp", "ScalingDown", "RotationR",
                      "RotationL"};

struct EulerSolverParam {
    double brush_positionx;
    double brush_positiony;
    double brush_deltax;
    double brush_deltay;
    double brush_stength;
    double damping;
    double radius;
};

struct ClassicSolverParam {
    double brush_positionx;
    double brush_positiony;
    double brush_deltax;
    double brush_deltay;
    double brush_stength;
    double radius;
    bool click;
    int editmode;

    double solver_fps;
};
