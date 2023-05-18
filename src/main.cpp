#include "viewer/rl_euler_editor.hpp"

#include "solver/euler.hpp"
#include "solver/grid2.hpp"

int main() {
    omp_set_nested(true);

    RLEulerEditor editor;
    editor.init("liquify editor");
    vector<Color> pixels(1200 * 900);
    for (int i = 0; i < 1200; i++) {
        for (int j = 0; j < 900; j++) {
            pixels[i + j * 1200].a = 255;
            pixels[i + j * 1200].r =
                255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
            pixels[i + j * 1200].g =
                255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
            pixels[i + j * 1200].b =
                255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
        }
    }

    int level = 10;
    EulerFluidSolver solver(level);
    solver.assign_field(pixels);
    solver.set_reset_buffer(
        [&](const vector<Color> &pixels) { editor.reset_buffer(pixels); });
    solver.set_update_buffer(
        [&](const vector<Color> &pixels) { editor.update_buffer(pixels); });

    const bool &is_closed = editor.is_closed();
#pragma omp parallel sections num_threads(2) default(shared)
    {
#pragma omp section
        { editor.run(); }
#pragma omp section
        { solver.run(is_closed); }
    }
    return 0;
}