#pragma once

#include "src/common.hpp"

enum EditMode { Trans, ScalingUp, ScalingDown, RotationR, RotationL };
string EditmodeStr[5]{"Trans", "ScalingUp", "ScalingDown", "RotationR",
                      "RotationL"};

struct EulerSolverParam {
    double brush_positionx;
    double brush_positiony;
    double brush_deltax;
    double brush_deltay;
    double brush_stength;
    double radius;
    double max_radius;
    double damping;
    bool click;
    int editmode;
    bool reset = false;

    double solver_fps;
    bool solver_reset = false;
};

struct ClassicSolverParam {
    double brush_positionx;
    double brush_positiony;
    double brush_deltax;
    double brush_deltay;
    double brush_stength;
    double radius;
    double max_radius;
    bool click;
    int editmode;
    bool reset = false;

    double solver_fps;
    bool solver_reset = false;
};
