#pragma once

#include "solver/common.hpp"
#include "solver/grid2.hpp"

class ClassicWarpingSolver {
  public:
    ClassicWarpingSolver(int level) {
        _resolution = Vector2i(_level_basex, _level_basey) * level;
        _origin = Vector2d::Zero();
        _grid_spacing = 1. / (_resolution.x() - 1);
        _density.resize(_resolution, _origin, _grid_spacing);
        _pixels.resize(_img_width * _img_height);
        _origin_pixels.resize(_img_width * _img_height);
    }
    void set_reset_buffer(const function<void(const vector<Color> &)> &func) {
        _reset_buffer = func;
    }
    void set_update_buffer(const function<void(const vector<Color> &)> &func) {
        _update_buffer = func;
    }
    void init_image(const vector<Color> &pixels) {
        for (int i = 0; i < _img_width; i++) {
            for (int j = 0; j < _img_height; j++) {
                _pixels[i + j * _img_width] = pixels[i + j * _img_width];
                _origin_pixels[i + j * _img_width] = pixels[i + j * _img_width];
            }
        }
        assign_field(pixels);
    }
    void assign_field(const vector<Color> &pixels) {
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

    void step() { advection(); }
    void run(const bool &is_closed, ClassicSolverParam &solver_param) {
        if (_reset_buffer != NULL) {
            assign_image();
            _reset_buffer(_pixels);
        }
        while (!is_closed) {
            double t0, t1;
            t0 = omp_get_wtime();
            _param = solver_param;
            step();
            if (_update_buffer != NULL) {
                static int update_count = 0;
                if ((update_count % _substep) == 0) {
                    assign_image();
                    _update_buffer(_pixels);
                }
                update_count += 1;
            }
            t1 = omp_get_wtime();
            if (1 / (t1 - t0) > _target_fps) {
                _sleep(ceil((1e3 / _target_fps) - (t1 - t0)));
            }
            t1 = omp_get_wtime();
            solver_param.solver_fps = 1 / (t1 - t0);
        }
    }

  protected:
    void reset_image() {
        for (int i = 0; i < _img_width; i++) {
            for (int j = 0; j < _img_height; j++) {
                _pixels[i + j * _img_width] =
                    _origin_pixels[i + j * _img_width];
            }
        }
    }
    void advection() {
        if (_param.reset) {
            reset_image();
            assign_field(_origin_pixels);
        }
        if (_param.click) {
            Grid2<Vector4d> oldd;
            oldd.clone(_density);

            Vector2d dmouse =
                Vector2d(_param.brush_deltax, _param.brush_deltay);
            Vector2d pmouse =
                Vector2d(_param.brush_positionx, _param.brush_positiony);
            Vector2i idxlow, idxup;
            double radius = _param.radius;
            double radius2 = radius * radius;
            double strength = _param.brush_stength;
            idxlow =
                _density.pos2idx(pmouse - radius * 1.05 * Vector2d::Ones());
            idxup = _density.pos2idx(pmouse + radius * 1.05 * Vector2d::Ones());

            int mode = _param.editmode;

            if (mode == Trans) {
                for (int i = idxlow.x(); i < idxup.x(); i++) {
                    for (int j = idxlow.y(); j < idxup.y(); j++) {
                        Vector2d px = _density.idx2pos(i, j);
                        double dx2 = (px - pmouse).squaredNorm();
                        if (dx2 < radius2) {
                            double fact = (radius2 - dx2) /
                                          ((radius2 - dx2) + dmouse.norm());
                            fact = fact * fact;
                            fact *= strength * _trans_strength_coeff;
                            Vector2d dpos =
                                fact * dmouse * (_param.max_radius / radius);
                            Vector2d srcpos = px - dpos;
                            _density(i, j) = oldd.linear_sample(srcpos);
                        }
                    }
                }
            } else if (mode == ScalingUp || mode == ScalingDown) {
                int sign = mode == ScalingUp ? 1 : -1;
                for (int i = idxlow.x(); i < idxup.x(); i++) {
                    for (int j = idxlow.y(); j < idxup.y(); j++) {
                        Vector2d px = _density.idx2pos(i, j);
                        double dx2 = (px - pmouse).squaredNorm();
                        double dx = (px - pmouse).norm();
                        if (dx2 < radius2) {
                            double fact = dx / radius - 1;
                            fact = fact * fact;
                            fact =
                                fact * sign * strength * _scale_strength_coeff;
                            fact = (1 - fact) * dx;
                            Vector2d dpos = fact * (px - pmouse).normalized();
                            Vector2d srcpos = pmouse + dpos;
                            _density(i, j) = oldd.linear_sample(srcpos);
                        }
                    }
                }
            } else if (mode == RotationR || mode == RotationL) {
                int sign = mode == RotationR ? 1 : -1;
                for (int i = idxlow.x(); i < idxup.x(); i++) {
                    for (int j = idxlow.y(); j < idxup.y(); j++) {
                        Vector2d px = _density.idx2pos(i, j);
                        double dx2 = (px - pmouse).squaredNorm();
                        double dx = (px - pmouse).norm();
                        if (dx2 < radius2) {
                            double fact = 1 - dx2 / radius2;
                            fact = fact * fact;
                            fact =
                                fact * sign * strength * _rotate_strength_coeff;

                            double theta =
                                atan2((px - pmouse).x(), (px - pmouse).y());
                            theta += fact;
                            Vector2d dpos =
                                Vector2d(sin(theta), cos(theta)) * dx;
                            Vector2d srcpos = pmouse + dpos;
                            _density(i, j) = oldd.linear_sample(srcpos);
                        }
                    }
                }
            }
        }
    }

  private:
    function<void(const vector<Color> &)> _reset_buffer;
    function<void(const vector<Color> &)> _update_buffer;
    function<void(ClassicSolverParam &)> _solver_param;

    int _substep = 1;
    int _target_fps = 60;

    int _level_basex = 12;
    int _level_basey = 9;
    int _img_width = 1200;
    int _img_height = 900;

    double _trans_strength_coeff = 0.05;
    double _scale_strength_coeff = 0.002;
    double _rotate_strength_coeff = 0.002;

    Vector2i _resolution;
    Vector2d _origin;
    double _grid_spacing;
    Grid2<Vector4d> _density;
    vector<Color> _pixels;
    vector<Color> _origin_pixels;
    ClassicSolverParam _param;
};
