#pragma once
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "json/json.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <sstream>

namespace fs = std::filesystem;

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using Jvalue = Json::Value;
using std::array;
using std::cout;
using std::endl;
using std::fixed;
using std::fstream;
using std::function;
using std::ifstream;
using std::make_shared;
using std::map;
using std::ofstream;
using std::queue;
using std::set;
using std::setfill;
using std::setprecision;
using std::setw;
using std::shared_ptr;
using std::sort;
using std::string;
using std::stringstream;
using std::vector;

inline constexpr float PIF = 3.14159265358979323846264338327950288f;
inline constexpr double PID = 3.14159265358979323846264338327950288;
