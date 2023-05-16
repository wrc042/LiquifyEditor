#pragma once

#include "src/common.hpp"

struct EulerSolverParam {
    double brush_positionx;
    double brush_positiony;
    double brush_deltax;
    double brush_deltay;
    double brush_stength;
    double damping;
    double radius;
};