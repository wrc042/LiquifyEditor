#pragma once

#include "solver/common.hpp"

struct StdKernel2 {
    double h, h2, h3, h4, h5;
    explicit StdKernel2(double radius)
        : h(radius), h2(h * h), h3(h * h2), h4(h2 * h2), h5(h2 * h3){};
    double operator()(double distance) const {
        double distance_squared = distance * distance;

        if (distance_squared >= h2) {
            return 0.0;
        } else {
            double x = 1.0 - distance_squared / h2;
            return 4.0 / (PID * h2) * x * x * x;
        }
    };
    double first_derivative(double distance) const {
        if (distance >= h) {
            return 0.0;
        } else {
            double x = 1.0 - distance * distance / h2;
            return -24.0 * distance / (PID * h4) * x * x;
        }
    };
    Vector2d gradient(const Vector2d &point) const {
        double dist = point.norm();
        if (dist > 0.0) {
            return -first_derivative(dist) * (point / dist);
        } else {
            return Vector2d(0, 0);
        }
    };
    Vector2d gradient(double distance, const Vector2d &direction) const {
        return -first_derivative(distance) * direction;
    }
    double second_derivative(double distance) const {
        double distance_squared = distance * distance;

        if (distance_squared >= h2) {
            return 0.0;
        } else {
            double x = distance_squared / h2;
            return 24.0 / (PID * h4) * (1 - x) * (5 * x - 1);
        }
    };
    ~StdKernel2(){};
};

struct SpikyKernel2 {
    double h, h2, h3, h4, h5;
    explicit SpikyKernel2(double radius)
        : h(radius), h2(h * h), h3(h * h2), h4(h2 * h2), h5(h2 * h3){};
    double operator()(double distance) const {
        if (distance >= h) {
            return 0.0;
        } else {
            double x = 1.0 - distance / h;
            return 10.0 / (PID * h2) * x * x * x;
        }
    };
    double first_derivative(double distance) const {
        if (distance >= h) {
            return 0.0;
        } else {
            double x = 1.0 - distance / h;
            return -30.0 / (PID * h3) * x * x;
        }
    }
    Vector2d gradient(const Vector2d &point) const {
        double dist = point.norm();
        if (dist > 0.0) {
            return -first_derivative(dist) * (point / dist);
        } else {
            return Vector2d(0, 0);
        }
    };
    Vector2d gradient(double distance, const Vector2d &direction) const {
        return -first_derivative(distance) * direction;
    }
    double second_derivative(double distance) const {
        if (distance >= h) {
            return 0.0;
        } else {
            double x = 1.0 - distance / h;
            return 60.0 / (PID * h4) * x;
        }
    };
    ~SpikyKernel2(){};
};
