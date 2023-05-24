#pragma once

#include "IO/common.hpp"

namespace IO {

// Jvalue load_json_config(string config_path) {
//     Jvalue config;
//     ifstream ifs;
//     ifs.open(config_path.c_str());
//     if (!ifs.is_open()) {
//         cout << "Error when loading config" << endl;
//         return config;
//     }
//     if (!Json::Reader().parse(ifs, config)) {
//         cout << "Error when parsing config" << endl;
//         return config;
//     }
//     return config;
// }

Ynode load_yaml_config(string config_path) {
    Ynode config = YAML::LoadFile(config_path);
    return config;
}

} // namespace IO
