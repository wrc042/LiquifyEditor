#pragma once

#include "viewer/common.hpp"

class ImageLoader {
  public:
    ImageLoader(string name, string resize_type)
        : _name(name), _resize_type(resize_type) {}
    void load(vector<Color> &pixels) {
        pixels.resize(_img_width * _img_height);
        for (int i = 0; i < _img_width * _img_height; i++) {
            pixels[i] = Color{0, 0, 0, 0};
        }
        cout << "Loading image: " << _name << endl;
        if (_name == "blocks") {
            load_block(pixels);
        } else if (_name.substr(_name.size() - 4, _name.size()) == ".png") {
            if (_resize_type == "fullimage") {
                load_image_full_image(pixels);
            } else {
                load_image_full_screen(pixels);
            }
        } else {
            cout << "Other image formats currently not support." << endl;
            load_block(pixels);
        }
    }

  protected:
    void load_block(vector<Color> &pixels) {
        for (int i = 0; i < _img_width; i++) {
            for (int j = 0; j < _img_height; j++) {
                pixels[i + j * _img_width].a = 255;
                pixels[i + j * _img_width].r =
                    255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
                pixels[i + j * _img_width].g =
                    255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
                pixels[i + j * _img_width].b =
                    255 * ((((i / 50) % 2) + ((j / 50) % 2)) % 2);
            }
        }
    }
    void load_image_full_image(vector<Color> &pixels) {
        if (!ifstream(_name.c_str()).good()) {
            cout << "Image not exists" << endl;
            load_block(pixels);
            return;
        }

        Image image = LoadImage(_name.c_str());

        int width = image.width;
        int height = image.height;

        double wscale = 1.0 * width / _img_width;
        double hscale = 1.0 * height / _img_height;

        double wdis = wscale;
        double hdis = hscale;

        int wafter, hafter;

        if (wdis > hdis) {
            wafter = _img_width;
            hafter = height / wscale;
        } else {
            hafter = _img_height;
            wafter = width / hscale;
        }
        cout << "Resize: " << wafter << "x" << hafter << endl;

        ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        ImageResize(&image, wafter, hafter);
        Color *image_pixels = LoadImageColors(image);

        if (wdis > hdis) {
            int padding = (_img_height - hafter + 1) / 2;
            for (int i = 0; i < _img_width; i++) {
                for (int j = padding; j < (_img_height - padding); j++) {
                    pixels[i + j * _img_width] =
                        image_pixels[i + (j - padding) * wafter];
                }
            }
        } else {
            int padding = (_img_width - wafter + 1) / 2;
            for (int i = padding; i < (_img_width - padding); i++) {
                for (int j = 0; j < _img_height; j++) {
                    pixels[i + j * _img_width] =
                        image_pixels[(i - padding) + j * wafter];
                }
            }
        }

        UnloadImageColors(image_pixels);
        UnloadImage(image);
    }
    void load_image_full_screen(vector<Color> &pixels) {
        if (!ifstream(_name.c_str()).good()) {
            cout << "Image not exists" << endl;
            load_block(pixels);
            return;
        }

        Image image = LoadImage(_name.c_str());

        int width = image.width;
        int height = image.height;

        double wscale = 1.0 * width / _img_width;
        double hscale = 1.0 * height / _img_height;

        double wdis = wscale;
        double hdis = hscale;

        int wafter, hafter;

        if (wdis < hdis) {
            wafter = _img_width;
            hafter = height / wscale;
        } else {
            hafter = _img_height;
            wafter = width / hscale;
        }
        cout << "Resize: " << wafter << "x" << hafter << endl;

        ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        ImageResize(&image, wafter, hafter);
        Color *image_pixels = LoadImageColors(image);

        if (wdis < hdis) {
            int padding = (hafter - _img_height + 1) / 2;
            for (int i = 0; i < _img_width; i++) {
                for (int j = 0; j < _img_height; j++) {
                    pixels[i + j * _img_width] =
                        image_pixels[i + (j + padding) * wafter];
                }
            }
        } else {
            int padding = (wafter - _img_width + 1) / 2;
            for (int i = 0; i < _img_width; i++) {
                for (int j = 0; j < _img_height; j++) {
                    pixels[i + j * _img_width] =
                        image_pixels[(i + padding) + j * wafter];
                }
            }
        }

        UnloadImageColors(image_pixels);
        UnloadImage(image);
    }

  private:
    string _name;
    string _resize_type;
    int _img_width = 1200;
    int _img_height = 900;
};
