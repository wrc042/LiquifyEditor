#include "solver/grid2.hpp"
#include "viewer/rl_euler_editor.hpp"

int main() {
    omp_set_nested(true);

    RLEulerEditor editor;
    editor.init("liquify editor");
    vector<Color> pixels(1200 * 900);
    for (int i = 0; i < pixels.size(); i++) {
        pixels[i].a = 255;
        pixels[i].r = 255;
        pixels[i].g = 255;
        pixels[i].b = 255;
    }
    int level = 20;
    Vector2i resolution = Vector2i(level * 12, level * 9);
    double grid_spacing = 1. / resolution.x();

    Image image = LoadImage("imgs/test.png");
    Color *pixels_ = LoadImageColors(image);
    for (int i = 0; i < pixels.size(); i++) {
        pixels[i] = pixels_[i];
    }
    Grid2<Vector4d> density(resolution, Vector2d::Zero(), grid_spacing);

    // image2field
    density.fill([&](Vector2d pos) {
        auto lerp = [](double low, double x, Vector4d v_low, Vector4d v_high)
        {
            double d = x - low;
            return (1 - d) * v_low + d * v_high;
        };
        auto visit = [&pixels](int x, int y) {
            double a = pixels[x + y * 1200].a;
            double r = pixels[x + y * 1200].r;
            double g = pixels[x + y * 1200].g;
            double b = pixels[x + y * 1200].b;
            return Vector4d(a, r, g, b);
        };

        Vector2d image_pos = pos * 1200;
        Vector2i points[2];
        points[0] =
            Vector2i(int(floor(image_pos.x())), int(floor(image_pos.y())));
        points[1] = points[0] + Vector2i::Ones();

        points[0].x() = (std::min)(points[0].x(), 1199);
        points[0].x() = (std::max)(points[0].x(), 0);
        points[1].x() = (std::min)(points[1].x(), 1199);
        points[1].x() = (std::max)(points[1].x(), 0);

        points[0].y() = (std::min)(points[0].y(), 899);
        points[0].y() = (std::max)(points[0].y(), 0);
        points[1].y() = (std::min)(points[1].y(), 899);
        points[1].y() = (std::max)(points[1].y(), 0);
        Vector4d result;
        Vector4d values[2];
        values[0] =
            lerp(points[0].x(), pos.x(), visit(points[0].x(), points[0].y()),
                 visit(points[1].x(), points[0].y()));
        values[1] =
            lerp(points[0].x(), pos.x(), visit(points[0].x(), points[1].y()),
                 visit(points[1].x(), points[1].y()));
        result = lerp(points[0].y(), pos.y(), values[0], values[1]);
        return result;
    });

    // density.fill([&](Vector2d pos) {
    //     Vector4d result;
    //     result(0) = pos.x() * 255;
    //     result(1) = pos.x() * 255;
    //     result(2) = pos.x() * 255;
    //     result(3) = pos.x() * 255;
    //     return result;
    // });

    // field2image
    for (int i = 0; i < 1200; i++) {
        for (int j = 0; j < 900; j++) {
            Vector2d pos = Vector2d(i, j) / 1200;
            Vector4d value = density.linear_sample(pos);
            pixels[i + j * 1200].a = value(0);
            pixels[i + j * 1200].r = value(1);
            pixels[i + j * 1200].g = value(2);
            pixels[i + j * 1200].b = value(3);
        }
    }

    const bool &is_closed = editor.is_closed();
#pragma omp parallel sections num_threads(2) default(shared)
    {
#pragma omp section
        { editor.run(); }
#pragma omp section
        {
            editor.reset_buffer(pixels);
            while (!is_closed) {
                editor.update_buffer(pixels);
            }
        }
    }
    return 0;
}