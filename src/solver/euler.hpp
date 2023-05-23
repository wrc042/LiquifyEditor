#pragma once

#include "solver/common.hpp"
#include "solver/grid2.hpp"

class EulerFluidSolver {
  public:
    EulerFluidSolver(int level) {
        _resolution = Vector2i(_level_basex, _level_basey) * level;
        _origin = Vector2d::Zero();
        _grid_spacing = 1. / (_resolution.x() - 1);
        _velocityu.resize(_resolution - Vector2i ::UnitX(),
                          _origin + Vector2d::UnitY() * _grid_spacing * 0.5,
                          _grid_spacing);
        _velocityv.resize(_resolution - Vector2i ::UnitY(),
                          _origin + Vector2d::UnitX() * _grid_spacing * 0.5,
                          _grid_spacing);
        _density.resize(_resolution, _origin, _grid_spacing);
        _pixels.resize(_img_width * _img_height);
        build_solver();
        clear_velocity();
        for (int i = _resolution.x() * 0.4; i < _resolution.x() * 0.6; i++) {
            for (int j = _resolution.y() * 0.4; j < _resolution.y() * 0.6;
                 j++) {
                _velocityu(i, j) = 1;
            }
        }
    }
    void set_reset_buffer(const function<void(const vector<Color> &)> &func) {
        _reset_buffer = func;
    }
    void set_update_buffer(const function<void(const vector<Color> &)> &func) {
        _update_buffer = func;
    }
    void assign_field(const vector<Color> &pixels) {
        for (int i = 0; i < _img_width; i++) {
            for (int j = 0; j < _img_height; j++) {
                _pixels[i + j * _img_width].a = pixels[i + j * _img_width].a;
                _pixels[i + j * _img_width].r = pixels[i + j * _img_width].r;
                _pixels[i + j * _img_width].g = pixels[i + j * _img_width].g;
                _pixels[i + j * _img_width].b = pixels[i + j * _img_width].b;
            }
        }
        _density.fill([&](Vector2d pos) {
            auto lerp = [](double low, double x, Vector4d v_low,
                           Vector4d v_high) {
                double d = x - low;
                return (1 - d) * v_low + d * v_high;
            };
            auto visit = [&](int x, int y) {
                double a = pixels[x + y * _img_width].a;
                double r = pixels[x + y * _img_width].r;
                double g = pixels[x + y * _img_width].g;
                double b = pixels[x + y * _img_width].b;
                return Vector4d(a, r, g, b);
            };

            Vector2d image_pos = pos * (_img_width - 1);
            Vector2i points[2];
            points[0] =
                Vector2i(int(floor(image_pos.x())), int(floor(image_pos.y())));
            points[1] = points[0] + Vector2i::Ones();

            points[0].x() = (std::min)(points[0].x(), (_img_width - 1));
            points[0].x() = (std::max)(points[0].x(), 0);
            points[1].x() = (std::min)(points[1].x(), (_img_width - 1));
            points[1].x() = (std::max)(points[1].x(), 0);

            points[0].y() = (std::min)(points[0].y(), (_img_height - 1));
            points[0].y() = (std::max)(points[0].y(), 0);
            points[1].y() = (std::min)(points[1].y(), (_img_height - 1));
            points[1].y() = (std::max)(points[1].y(), 0);
            Vector4d result;
            Vector4d values[2];
            values[0] = lerp(points[0].x(), image_pos.x(),
                             visit(points[0].x(), points[0].y()),
                             visit(points[1].x(), points[0].y()));

            values[1] = lerp(points[0].x(), image_pos.x(),
                             visit(points[0].x(), points[1].y()),
                             visit(points[1].x(), points[1].y()));

            result = lerp(points[0].y(), image_pos.y(), values[0], values[1]);
            return result;
        });
    }
    void assign_image() { assign_image(_pixels); }
    void assign_image(vector<Color> &pixels) {
        auto color_clamp = [](double x) {
            int c = floor(x);
            c = (std::min)(c, 255);
            c = (std::max)(c, 0);
            return c;
        };
        for (int i = 0; i < _img_width; i++) {
            for (int j = 0; j < _img_height; j++) {
                Vector2d pos = Vector2d(i + 0.5, j + 0.5) / _img_width;
                Vector4d value = _density.linear_sample(pos);
                pixels[i + j * _img_width].a = color_clamp(value(0));
                pixels[i + j * _img_width].r = color_clamp(value(1));
                pixels[i + j * _img_width].g = color_clamp(value(2));
                pixels[i + j * _img_width].b = color_clamp(value(3));
            }
        }
    }

    void step() {
        advection(_time_interval);
        projection();
    }
    void run(const bool &is_closed) {
        if (_reset_buffer != NULL) {
            assign_image();
            _reset_buffer(_pixels);
        }
        while (!is_closed) {
            if (_sleep_interval != 0) {
                _sleep(_sleep_interval);
            }
            step();
            if (_update_buffer != NULL) {
                static int update_count = 0;
                if ((update_count % _substep) == 0) {
                    assign_image();
                    _update_buffer(_pixels);
                }
                update_count += 1;
            }
        }
    }

  protected:
    void build_solver() {
        typedef Eigen::Triplet<double> T;
        int n = _resolution.prod();
        _projectoin_matrix.resize(n, n);
        int offset = _resolution.y();
        std::vector<T> triple_list;

        int count = 0;
        for (int i = 0; i < _resolution.x(); i++) {
            for (int j = 0; j < _resolution.y(); j++) {
                int coeff = 0;
                if (i > 0) {
                    triple_list.push_back(T(count, (i - 1) * offset + j, -1));
                    coeff += 1;
                }
                if (i + 1 < _resolution.x()) {
                    triple_list.push_back(T(count, (i + 1) * offset + j, -1));
                    coeff += 1;
                }
                if (j > 0) {
                    triple_list.push_back(T(count, i * offset + j - 1, -1));
                    coeff += 1;
                }
                if (j + 1 < _resolution.y()) {
                    triple_list.push_back(T(count, i * offset + j + 1, -1));
                    coeff += 1;
                }
                triple_list.push_back(T(count, count, coeff));
                count++;
            }
        }
        _projectoin_matrix.setFromTriplets(triple_list.begin(),
                                           triple_list.end());
        // _cg_sovler.compute(_projectoin_matrix);
        // _icpcg_sovler.compute(_projectoin_matrix);
        _lu_solver.compute(_projectoin_matrix);
    }
    void clear_velocity() {
        _velocityu.fill([](Vector2d pos) { return 0.0; });
        _velocityv.fill([](Vector2d pos) { return 0.0; });
    }
    void advection(double time_interval) {
        Grid2<double> oldu;
        oldu.clone(_velocityu);
        Grid2<double> oldv;
        oldv.clone(_velocityv);
        Grid2<Vector4d> oldd;
        oldd.clone(_density);

        for (int i = 0; i < _velocityu.resolution().x(); i++) {
            for (int j = 0; j < _velocityu.resolution().y(); j++) {
                Vector2d pos = back_trace(oldu, oldv, time_interval,
                                          _velocityu.idx2pos(i, j));
                _velocityu(i, j) = oldu.linear_sample(pos);
            }
        }

        for (int i = 0; i < _velocityv.resolution().x(); i++) {
            for (int j = 0; j < _velocityv.resolution().y(); j++) {
                Vector2d pos = back_trace(oldu, oldv, time_interval,
                                          _velocityv.idx2pos(i, j));
                _velocityv(i, j) = oldv.linear_sample(pos);
            }
        }

        for (int i = 0; i < _density.resolution().x(); i++) {
            for (int j = 0; j < _density.resolution().y(); j++) {
                Vector2d pos = back_trace(oldu, oldv, time_interval,
                                          _density.idx2pos(i, j));
                _density(i, j) = oldd.linear_sample(pos);
            }
        }
    }
    Vector2d back_trace(const Grid2<double> &velocityu,
                        const Grid2<double> &velocityv, double time_interval,
                        const Vector2d &pt_start) {
        // double vu = velocityu.linear_sample(pt_start);
        // double vv = velocityv.linear_sample(pt_start);
        // return pt_start - time_interval * Vector2d(vu, vv);
        double vu = mcr_sample(velocityu, pt_start);
        double vv = mcr_sample(velocityv, pt_start);
        Vector2d midpoint = pt_start - 0.5 * time_interval * Vector2d(vu, vv);
        double midvu = mcr_sample(velocityu, midpoint);
        double midvv = mcr_sample(velocityv, midpoint);
        return pt_start - time_interval * Vector2d(midvu, midvv);
    }
    void projection() {
        int n = _resolution.prod();
        int offset = _resolution.y();
        Eigen::VectorXd b(n);
        Eigen::VectorXd p(n);

        int count = 0;
        for (int i = 0; i < _resolution.x(); i++) {
            for (int j = 0; j < _resolution.y(); j++) {
                b(count) =
                    -_grid_spacing * (_velocityu(i + 1, j) - _velocityu(i, j) +
                                      _velocityv(i, j + 1) - _velocityv(i, j));
                if (i == 0) {
                    b(count) -= _grid_spacing * _velocityu(i, j);
                }
                if (i + 1 == _resolution.x()) {
                    b(count) -= -_grid_spacing * _velocityu(i + 1, j);
                }
                if (j == 0) {
                    b(count) -= _grid_spacing * _velocityv(i, j);
                }
                if (j + 1 == _resolution.y()) {
                    b(count) -= -_grid_spacing * _velocityv(i, j + 1);
                }
                count++;
            }
        }
        // p = _cg_sovler.solve(b);
        // p = _icpcg_sovler.solve(b);
        p = _lu_solver.solve(b);

        for (int i = 0; i < _velocityu.resolution().x(); i++) {
            for (int j = 0; j < _velocityu.resolution().y(); j++) {
                if (i == 0 || i == _velocityu.resolution().x() - 1) {
                    _velocityu(i, j) = 0;
                    continue;
                }
                _velocityu(i, j) -=
                    (p(i * offset + j) - p((i - 1) * offset + j)) /
                    _grid_spacing;
            }
        }

        for (int i = 0; i < _velocityv.resolution().x(); i++) {
            for (int j = 0; j < _velocityv.resolution().y(); j++) {
                if (j == 0 || j == _velocityv.resolution().y() - 1) {
                    _velocityv(i, j) = 0;
                    continue;
                }
                _velocityv(i, j) -=
                    (p(i * offset + j) - p(i * offset + j - 1)) / _grid_spacing;
            }
        }
    }

  private:
    function<void(const vector<Color> &)> _reset_buffer;
    function<void(const vector<Color> &)> _update_buffer;

    int _substep = 1;
    int _sleep_interval = 0;

    double _time_interval = 5e-3;

    int _level_basex = 12;
    int _level_basey = 9;
    int _img_width = 1200;
    int _img_height = 900;

    Vector2i _resolution;
    Vector2d _origin;
    double _grid_spacing;
    Grid2<double> _velocityu;
    Grid2<double> _velocityv;
    Grid2<Vector4d> _density;
    vector<Color> _pixels;
    Eigen::SparseMatrix<double> _projectoin_matrix;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
                             Eigen::Lower | Eigen::Upper>
        _cg_sovler;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
                             Eigen::Lower | Eigen::Upper,
                             Eigen::IncompleteCholesky<double>>
        _icpcg_sovler;
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
        _lu_solver;
};
