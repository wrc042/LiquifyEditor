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

    const bool &is_closed = editor.is_closed();
#pragma omp parallel sections num_threads(2) default(shared)
    {
#pragma omp section
        { editor.run(); }
#pragma omp section
        {
            editor.reset_buffer(pixels);
            int step = 0;

            while (!is_closed) {
                step++;
                for (int i = 0; i < pixels.size(); i++) {
                    if (abs(i / 1200 - step % 900) < 20) {
                        pixels[i].a = 255;
                        pixels[i].r = 0;
                        pixels[i].g = 0;
                        pixels[i].b = 0;
                    } else if (abs(i % 1200 - step % 1200) < 20) {
                        pixels[i].a = 255;
                        pixels[i].r = 0;
                        pixels[i].g = 0;
                        pixels[i].b = 0;
                    } else {
                        pixels[i].a = 255;
                        pixels[i].r = 255;
                        pixels[i].g = 255;
                        pixels[i].b = 255;
                    }
                }
                editor.update_buffer(pixels);
            }
        }
    }
    return 0;
}