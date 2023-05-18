#pragma once

#include "solver/common.hpp"

template <typename T> class Grid2 {
  public:
    Grid2() {}
    Grid2(Vector2i resolution, Vector2d origin, double grid_spacing)
        : _resolution(resolution), _origin(origin),
          _grid_spacing(grid_spacing) {
        _data.resize(resolution.prod());
    }
    void resize(Vector2i resolution, Vector2d origin, double grid_spacing) {
        _resolution = resolution;
        _origin = origin;
        _grid_spacing = grid_spacing;
        _data.resize(resolution.prod());
    }
    const Vector2i &resolution() const { return _resolution; }
    const Vector2d &origin() const { return _origin; }
    const double &grid_spacing() const { return _grid_spacing; }
    void bound(Vector2i &idx) const {
        idx.x() = (std::max)(0, idx.x());
        idx.y() = (std::max)(0, idx.y());
        idx.x() = (std::min)(resolution().x() - 1, idx.x());
        idx.y() = (std::min)(resolution().y() - 1, idx.y());
    }
    const T &operator()(size_t i, size_t j) const { return _visit(i, j); }
    T &operator()(size_t i, size_t j) { return _visit(i, j); }
    const T &operator()(Vector2i idx) const { return _visit(idx.x(), idx.y()); }
    T &operator()(Vector2i idx) { return _visit(idx.x(), idx.y()); }
    Vector2d idx2pos(size_t i, size_t j) {
        return origin() + Vector2d(i * grid_spacing(), j * grid_spacing());
    }
    bool in_bound(size_t i, size_t j) {
        return i >= 0 && i < _resolution.x() && j >= 0 && j < _resolution.y();
    }
    Vector2d idx2pos(Vector2i idx) { return idx2pos(idx.x(), idx.y()); }
    bool in_bound(Vector2i idx) { return in_bound(idx.x(), idx.y()); }
    Vector2i pos2idx(Vector2d pos) {
        pos = (pos - origin()) / grid_spacing();
        Vector2i posi(int(floor(pos.x())), int(floor(pos.y())));
        bound(posi);
        return posi;
    }
    T linear_sample(Vector2d pos) const {
        auto lerp = [](double low, double x, T v_low, T v_high) {
            double d = x - low;
            return (1 - d) * v_low + d * v_high;
        };
        pos = (pos - origin()) / grid_spacing();
        Vector2i points[2];
        points[0] = Vector2i(int(floor(pos.x())), int(floor(pos.y())));
        points[1] = points[0] + Vector2i::Ones();
        bound(points[0]);
        bound(points[1]);
        T values[2];
        values[0] =
            lerp(points[0].x(), pos.x(), _visit(points[0].x(), points[0].y()),
                 _visit(points[1].x(), points[0].y()));
        values[1] =
            lerp(points[0].x(), pos.x(), _visit(points[0].x(), points[1].y()),
                 _visit(points[1].x(), points[1].y()));
        T result = lerp(points[0].y(), pos.y(), values[0], values[1]);
        return result;
    }
    void fill(const std::function<T(Vector2d)> &func) {
        for (size_t i = 0; i < resolution().x(); i++) {
            for (size_t j = 0; j < resolution().y(); j++) {
                _visit(i, j) = func(idx2pos(i, j));
            }
        }
    };
    void clone(const Grid2<T> &grid) {
        resize(grid._resolution, grid._origin, grid._grid_spacing);
        for (int i = 0; i < _resolution.prod(); i++) {
            _data[i] = grid._data[i];
        }
    }
    template <typename Callback> void foreach (Callback func) {
        for (int i = 0; i < _resolution.x(); i++) {
            for (int j = 0; j < _resolution.y(); j++) {
                func(i, j);
            }
        }
    }

  private:
    T &_visit(size_t i, size_t j) { return _data[i * resolution().y() + j]; }
    const T &_visit(size_t i, size_t j) const {
        return _data[i * resolution().y() + j];
    }
    std::vector<T> _data;
    Vector2i _resolution;
    Vector2d _origin;
    double _grid_spacing;
};

double monotonic_catmull_rom(const double &f0, const double &f1,
                             const double &f2, const double &f3, double f) {
    double d1 = (f2 - f0) / 2;
    double d2 = (f3 - f1) / 2;
    double D1 = f2 - f1;
    if (abs(D1) < 1e-6) {
        d1 = 0;
        d2 = 0;
    }
    if (D1 * d1 < 0) {
        d1 = 0;
    }
    if (D1 * d2 < 0) {
        d2 = 0;
    }
    double a3 = d1 + d2 - 2 * D1;
    double a2 = 3 * D1 - 2 * d1 - d2;
    double a1 = d1;
    double a0 = f1;

    return a3 * f * f * f + a2 * f * f + a1 * f + a0;
}

double mcr_sample(const Grid2<double> &grid, const Vector2d &pos) {
    Vector2d indexd = (pos - grid.origin()) / grid.grid_spacing();
    Vector2i index0(int(floor(indexd.x())) - 1, int(floor(indexd.y())) - 1);
    Vector2i index1 = index0 + Vector2i::Ones();
    Vector2i index2 = index0 + 2 * Vector2i::Ones();
    Vector2i index3 = index0 + 3 * Vector2i::Ones();
    indexd -= Vector2d(floor(indexd.x()), floor(indexd.y()));
    grid.bound(index0);
    grid.bound(index1);
    grid.bound(index2);
    grid.bound(index3);
    double tmp[4];
    tmp[0] = monotonic_catmull_rom(
        grid(index0.x(), index0.y()), grid(index1.x(), index0.y()),
        grid(index2.x(), index0.y()), grid(index3.x(), index0.y()), indexd.x());
    tmp[1] = monotonic_catmull_rom(
        grid(index0.x(), index1.y()), grid(index1.x(), index1.y()),
        grid(index2.x(), index1.y()), grid(index3.x(), index1.y()), indexd.x());
    tmp[2] = monotonic_catmull_rom(
        grid(index0.x(), index2.y()), grid(index1.x(), index2.y()),
        grid(index2.x(), index2.y()), grid(index3.x(), index2.y()), indexd.x());
    tmp[3] = monotonic_catmull_rom(
        grid(index0.x(), index3.y()), grid(index1.x(), index3.y()),
        grid(index2.x(), index3.y()), grid(index3.x(), index3.y()), indexd.x());
    double result =
        monotonic_catmull_rom(tmp[0], tmp[1], tmp[2], tmp[3], indexd.y());
    return result;
}
