#include "IO/config.hpp"

#include "viewer/image.hpp"
#include "viewer/rl_classic_editor.hpp"
#include "viewer/rl_euler_editor.hpp"

#include "solver/classic.hpp"
#include "solver/euler.hpp"
#include "solver/grid2.hpp"

int main() {
    omp_set_nested(true);

    auto config = IO::load_config("config.json");

    RLClassicEditor editor;
    editor.init("liquify editor");
    vector<Color> origin_pixels;
    ImageLoader(config["image"].asString()).load(origin_pixels);

    ClassicWarpingSolver solver(config["level"].asInt());
    solver.assign_field(origin_pixels);
    solver.set_reset_buffer(
        [&](const vector<Color> &pixels) { editor.reset_buffer(pixels); });
    solver.set_update_buffer(
        [&](const vector<Color> &pixels) { editor.update_buffer(pixels); });

    const bool &is_closed = editor.is_closed();
    ClassicSolverParam &solver_param = editor.solver_param();
#pragma omp parallel sections num_threads(2) default(shared)
    {
#pragma omp section
        { editor.run(); }
#pragma omp section
        { solver.run(is_closed, solver_param); }
    }
    return 0;
}