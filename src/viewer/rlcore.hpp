#pragma once

#include "viewer/common.hpp"

#include "omp.h"

class RLCore {
  public:
    void init(string name) {
        InitWindow(_width, _height, name.c_str());
        omp_init_lock(&_buflock);
    }
    void run() {
        while (!WindowShouldClose()) {
            rl_loop();
        };
    }

  protected:
    virtual void init_callback(int bufnum) = 0;
    virtual void update_callback(int bufnum) = 0;
    virtual void gui_callback(){};
    virtual void save_callback(){};
    void rl_loop() {
        BeginDrawing();
        gui_callback();

        array<int, 2> bufnum;
        bool is_reset;

        get_bufnum(bufnum, is_reset);

        if (is_reset) {
            if (bufnum[0] != bufnum[1]) {
                init_callback(bufnum[1]);
            }
        } else {
            if (bufnum[0] != bufnum[1]) {
                save_callback();
                update_callback(bufnum[1]);
            }
        }
        bufnum[0] = bufnum[1];
        is_reset = false;

        set_bufnum(bufnum, is_reset);
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

    bool _is_reset = false;

  private:
    int _width = 1280;
    int _height = 720;

    array<int, 2> _bufnum = {0, 0};
    omp_lock_t _buflock;
};
