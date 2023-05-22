#pragma once
#include "viewer/rlcore.hpp"

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
        }
        set_bufnum(bufnum, is_reset);
    }
    const bool &is_closed() { return _is_closed; }

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
    void update_callback(int bufnum) {
        DrawRectangle(_origin.x(), _origin.y(), _range.x(), _range.y(), BLACK);
        DrawTexture(_textures[bufnum], _origin.x(), _origin.y(), WHITE);

        int text_height_cnt = 0;

        Vector2 brush_position = GetMousePosition();
        brush_position.x = (brush_position.x - _origin.x()) / _range.x();
        brush_position.x = (std::max)(brush_position.x, 0.f);
        brush_position.x = (std::min)(brush_position.x, 1.f);
        brush_position.y = (brush_position.y - _origin.y()) / _range.y();
        brush_position.y = (std::max)(brush_position.y, 0.f);
        brush_position.y = (std::min)(brush_position.y, 1.f);

        _brush_delta.x() = brush_position.x - _last_brush_position.x();
        _brush_delta.y() = brush_position.y - _last_brush_position.y();

        if (_brush_delta.norm() > _brush_delta_thres) {
            _brush_delta = Vector2d::Zero();
        }

        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            _brush_delta = Vector2d::Zero();
        }

        string text_delta = "Delta: ";
        text_delta += to_string(_brush_delta.x());
        text_delta += " ";
        text_delta += to_string(_brush_delta.y());

        DrawText(text_delta.c_str(), _text_left_padding,
                 _text_height * (++text_height_cnt), _text_font_size, BLACK);

        _last_brush_position.x() = brush_position.x;
        _last_brush_position.y() = brush_position.y;

        string text_position = "Position: ";
        text_position += to_string(brush_position.x);
        text_position += " ";
        text_position += to_string(brush_position.y);

        DrawText(text_position.c_str(), _text_left_padding,
                 _text_height * (++text_height_cnt), _text_font_size, BLACK);

        if (GetMouseWheelMove() > 0) {
            _brush_radius *= _brush_scale_ratio;
        } else if (GetMouseWheelMove() < 0) {
            _brush_radius /= _brush_scale_ratio;
        };
        _brush_radius = (std::max)(_brush_radius, _brush_radius_min);
        _brush_radius = (std::min)(_brush_radius, _brush_radius_max);

        DrawCircleLines(_last_brush_position.x() * _range.x() + _origin.x(),
                        _last_brush_position.y() * _range.y() + _origin.y(),
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

        _dampling =
            GuiSlider(Rectangle{float(_text_left_padding),
                                float(_text_height * (++text_height_cnt)),
                                float(_slider_width), float(_slider_height)},
                      NULL, TextFormat("Damping: %0.2e", _dampling), _dampling,
                      _dampling_min, _dampling_max);
    };

    void update_sovler_param() {
        solver_param.brush_positionx = _last_brush_position.x();
        solver_param.brush_positiony = _last_brush_position.y();
        solver_param.brush_deltax = _brush_delta.x();
        solver_param.brush_deltay = _brush_delta.y();
        solver_param.brush_stength = _strength;
        solver_param.damping = _dampling;
        solver_param.radius = _brush_radius;
    }

  private:
    Vector2d _last_brush_position = Vector2d(0, 0);
    Vector2d _brush_delta = Vector2d(0, 0);
    double _brush_delta_thres = 30;
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
    double _brush_radius_max = 100;

    double _strength = 50;
    double _strength_min = 0;
    double _strength_max = 100;

    double _dampling = 0;
    double _dampling_min = 0;
    double _dampling_max = 1e-3;

    EulerSolverParam solver_param;

    array<Texture, 2> _textures;
    array<Image, 2> _images;
    array<vector<Color>, 2> _pixel_buffers;
};
