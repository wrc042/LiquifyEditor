#pragma once

#include "viewer/common.hpp"

#include "omp.h"

class RLCore {
  public:
    void init(string name) {
        InitWindow(_width, _height, name.c_str());
        SetTargetFPS(60);
        omp_init_lock(&_buflock);
        on_init();
    }
    void run() {
        while (!WindowShouldClose()) {
            rl_loop();
        };
        _is_closed = true;
    }

  protected:
    virtual void init_callback(int bufnum) = 0;
    virtual void texture_callback(int bufnum){};
    virtual void update_callback(int bufnum) = 0;
    virtual void gui_callback(){};
    virtual void on_init(){};
    virtual void save_callback(){};
    void rl_loop() {
        array<int, 2> bufnum;
        bool is_reset;

        omp_set_lock(&_buflock);
        bufnum = _bufnum;
        is_reset = _is_reset;

        if (!is_reset) {
            texture_callback(bufnum[1]);
        }
        BeginDrawing();
        ClearBackground(RAYWHITE);
        gui_callback();

        if (is_reset) {
            init_callback(bufnum[1]);
            if (bufnum[0] != bufnum[1]) {
                is_reset = false;
            }
        } else {
            update_callback(bufnum[1]);
        }
        bufnum[0] = bufnum[1];

        _bufnum = bufnum;
        _is_reset = is_reset;
        omp_unset_lock(&_buflock);
        DrawFPS(3, 0);
        EndDrawing();
    }
    void get_bufnum(array<int, 2> &bufnum, bool &is_reset) {
        omp_set_lock(&_buflock);
        bufnum = _bufnum;
        is_reset = _is_reset;
        omp_unset_lock(&_buflock);
    }
    void set_bufnum(array<int, 2> &bufnum, bool &is_reset) {
        omp_set_lock(&_buflock);
        _bufnum = bufnum;
        _is_reset = is_reset;
        omp_unset_lock(&_buflock);
    }
    void init_bufnum(array<int, 2> &bufnum) {
        bufnum[1] = 1;
        bufnum[0] = 0;
    }
    void update_bufnum(array<int, 2> &bufnum) {
        bufnum[1] = (bufnum[1] == 0) ? 1 : 0;
    }
    bool to_update(array<int, 2> &bufnum) { return bufnum[0] == bufnum[1]; }

    bool _is_closed = false;
    bool _is_reset = true;

  private:
    int _width = 1600;
    int _height = 900;

    array<int, 2> _bufnum = {0, 0};
    omp_lock_t _buflock;
};
