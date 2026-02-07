#pragma once

#include <Eigen/Dense>

//Make these parameters!!!!
//----------------------
inline extern const double base = 0.38;
inline extern const double l1 = 0.62; 
inline extern const double l2 = 0.7;
inline extern const double l3 = 0.85;
inline extern const double l4 = 0.3;
inline extern const double l5 = 0.45;
//---------------------

inline extern const double PI = 3.14159265358979323846f;

inline extern const Eigen::Vector3d base_link(0, 0,base);
inline extern const Eigen::Vector3d link1(0, 0, l1);
inline extern const Eigen::Vector3d link2(l2, 0, 0);
inline extern const Eigen::Vector3d link3(l3, 0, 0);

inline extern const Eigen::Vector3d link4(l4, 0,0);
inline extern const Eigen::Vector3d link5(0, 0, -l5);
