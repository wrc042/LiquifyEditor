#pragma once

#include "IO/common.hpp"

namespace IO {

string gen_rid() {
    std::random_device rd;
    auto seed{rd()};
    std::default_random_engine rng(seed);

    std::uniform_int_distribution<int> dist(0, 15);

    const char *v = "0123456789abcdef";
    const bool dash[] = {0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0};

    string res;
    for (int i = 0; i < 16; i++) {
        if (dash[i])
            res += "-";
        res += v[dist(rng)];
        res += v[dist(rng)];
    }
    return res;
}

} // namespace IO
