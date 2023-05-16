#pragma once

#include "geom/common.hpp"

class SDFBox2 {
  public:
    function<double(Vector2d)> box2d(Vector2d &origin, Vector2d &range) {
        return [&](Vector2d x) {
            x = x - origin;
            x = Vector2d(std::abs(x.x()), std::abs(x.y()));
            Vector2d q = x - range;
            Vector2d q_ = Vector2d(std::max(q.x(), 0.0), std::max(q.y(), 0.0));
            return q_.norm() + std::min(std::max(q.x(), q.y()), 0.0);
        };
    }
};
