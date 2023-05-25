#pragma once
#include "viewer/rlcore.hpp"

#include "IO/uid.hpp"

class RLClassicEditor : public RLCore {
  public:
    void reset_buffer(const vector<Color> &pixels) { reset_buffer(&pixels[0]); }
    void reset_buffer(const Color *pixels) {
        array<int, 2> bufnum;
        bool is_reset;
        get_bufnum(bufnum, is_reset);
        init_bufnum(bufnum);
        // image operation
        for (int i = 0; i < _range.prod(); i++) {
            _pixel_buffers[bufnum[1]][i] = pixels[i];
        }
        is_reset = true;
        set_bufnum(bufnum, is_reset);
    }
    void update_buffer(const vector<Color> &pixels) {
        update_buffer(&pixels[0]);
    }
    void update_buffer(const Color *pixels) {
        array<int, 2> bufnum;
        bool is_reset;
        get_bufnum(bufnum, is_reset);
        if (to_update(bufnum)) {
            // image operation
            for (int i = 0; i < _range.prod(); i++) {
                _pixel_buffers[bufnum[1]][i] = pixels[i];
            }
            update_bufnum(bufnum);
            set_bufnum(bufnum, is_reset);
        }
    }
    const bool &is_closed() { return _is_closed; }
    ClassicSolverParam &solver_param() { return _solver_param; }

  protected:
    void on_init() {
        _pixel_buffers[0].resize(_range.prod());
        _pixel_buffers[1].resize(_range.prod());

        _images[0].data = &_pixel_buffers[0][0];
        _images[0].width = _range.x();
        _images[0].height = _range.y();
        _images[0].format = IMAGE_FORMAT;
        _images[0].mipmaps = 1;

        _images[1].data = &_pixel_buffers[1][0];
        _images[1].width = _range.x();
        _images[1].height = _range.y();
        _images[1].format = IMAGE_FORMAT;
        _images[1].mipmaps = 1;

        _textures[0] = LoadTextureFromImage(_images[0]);
        _textures[1] = LoadTextureFromImage(_images[1]);
    }
    void texture_callback(int bufnum) {
        UpdateTexture(_textures[bufnum], &_pixel_buffers[bufnum][0]);
    }
    void init_callback(int bufnum) {
        DrawText("Initializing...", 700, 450, 30, BLACK);
    };
    void save_image(int bufnum) {
        if (!fs::exists(_saving_dir)) {
            fs::create_directories(_saving_dir);
        }
        string name = IO::gen_rid();
        string path = _saving_dir + name + ".png";
        Image image = LoadImageFromTexture(_textures[bufnum]);
        ExportImage(image, path.c_str());
        UnloadImage(image);
    }
    void update_callback(int bufnum) {
        DrawTexture(_textures[bufnum], _origin.x(), _origin.y(), WHITE);

        int text_height_cnt = 0;

        Vector2 brush_position = GetMousePosition();
        Vector2 brush_delta = GetMouseDelta();
        _brush_click = IsMouseButtonDown(MOUSE_BUTTON_LEFT);

        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            _mode = (_mode + 1) % 5;
        }

        double dx = brush_position.x - _origin.x();
        double dy = brush_position.y - _origin.y();
        _brush_position.x() = dx / _range.x();
        _brush_position.y() = dy / _range.x();
        if (dx < 0 || dx > _range.x()) {
            _brush_click = false;
            _brush_position.x() = 0;
        }

        if (dy < 0 || dy > _range.y()) {
            _brush_click = false;
            _brush_position.y() = 0;
        }

        _brush_delta.x() = brush_delta.x / _range.x();
        _brush_delta.y() = brush_delta.y / _range.x();
        if (_brush_delta.norm() > _brush_delta_thres) {
            _brush_delta = Vector2d::Zero();
        }

        DrawText(TextFormat("Solver FPS: %0.2f", _solver_fps),
                 _text_left_padding, _text_height * (++text_height_cnt),
                 _text_font_size, BLACK);

        DrawText(EditmodeStr[_mode].c_str(), _text_left_padding,
                 _text_height * (++text_height_cnt), _text_font_size, BLACK);

        // string text_delta = "Delta: ";
        // text_delta += to_string(_brush_delta.x());
        // text_delta += " ";
        // text_delta += to_string(_brush_delta.y());

        // DrawText(text_delta.c_str(), _text_left_padding,
        //          _text_height * (++text_height_cnt), _text_font_size, BLACK);

        // string text_position = "Position: ";
        // text_position += to_string(_brush_position.x());
        // text_position += " ";
        // text_position += to_string(_brush_position.y());

        // DrawText(text_position.c_str(), _text_left_padding,
        //          _text_height * (++text_height_cnt), _text_font_size, BLACK);

        if (GetMouseWheelMove() > 0) {
            _brush_radius *= _brush_scale_ratio;
        } else if (GetMouseWheelMove() < 0) {
            _brush_radius /= _brush_scale_ratio;
        };
        _brush_radius = (std::max)(_brush_radius, _brush_radius_min);
        _brush_radius = (std::min)(_brush_radius, _brush_radius_max);

        DrawCircleLines(_brush_position.x() * _range.x() + _origin.x(),
                        _brush_position.y() * _range.x() + _origin.y(),
                        _brush_radius, WHITE);

        string text_radius = "Raduis: ";
        text_radius += to_string(_brush_radius);
        DrawText(text_radius.c_str(), _text_left_padding,
                 _text_height * (++text_height_cnt), _text_font_size, BLACK);

        _strength =
            GuiSlider(Rectangle{float(_text_left_padding),
                                float(_text_height * (++text_height_cnt)),
                                float(_slider_width), float(_slider_height)},
                      NULL, TextFormat("Strength: %0.2f", _strength), _strength,
                      _strength_min, _strength_max);

        _reset =
            GuiButton(Rectangle{float(_text_left_padding),
                                float(_text_height * (++text_height_cnt)),
                                float(_slider_width), float(_slider_height)},
                      "reset");

        if (GuiButton(Rectangle{float(_text_left_padding),
                                float(_text_height * (++text_height_cnt)),
                                float(_slider_width), float(_slider_height)},
                      "save")) {
            save_image(bufnum);
        }
        update_sovler_param();
    };

    void update_sovler_param() {
        _solver_param.brush_positionx = _brush_position.x();
        _solver_param.brush_positiony = _brush_position.y();
        _solver_param.brush_deltax = _brush_delta.x();
        _solver_param.brush_deltay = _brush_delta.y();
        _solver_param.brush_stength = _strength;
        _solver_param.radius = _brush_radius / _range.x();
        _solver_param.max_radius = _brush_radius_max / _range.x();
        _solver_param.click = _brush_click;
        _solver_param.editmode = _mode;
        _solver_param.reset = _reset;

        _solver_fps = _solver_param.solver_fps;
    }

  private:
    Vector2d _brush_position = Vector2d(0, 0);
    Vector2d _brush_delta = Vector2d(0, 0);
    double _brush_delta_thres = 0.3;
    Vector2i _origin = Vector2i(400, 0);
    Vector2i _range = Vector2i(1200, 900);

    int _text_height = 30;
    int _text_left_padding = 20;
    int _text_font_size = 20;
    int _slider_width = 250;
    int _slider_height = 20;

    double _brush_radius = 50;
    double _brush_scale_ratio = 1.2;
    double _brush_radius_min = 20;
    double _brush_radius_max = 150;
    bool _brush_click = false;

    double _strength = 50;
    double _strength_min = 0;
    double _strength_max = 100;

    bool _reset = false;

    double _solver_fps = 0;

    int _mode = 0;

    ClassicSolverParam _solver_param;

    array<Texture, 2> _textures;
    array<Image, 2> _images;
    array<vector<Color>, 2> _pixel_buffers;

    string _saving_dir = "output/";
};
