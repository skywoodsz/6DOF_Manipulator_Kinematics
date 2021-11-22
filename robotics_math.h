//
// Created by skywoodsz on 2021/11/22.
//

#ifndef SRC_CPP_ROBOTICS_MATH_H
#define SRC_CPP_ROBOTICS_MATH_H
#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

bool AxisAngle2Rorarion(Eigen::Vector3d w, double angle, Eigen::Matrix3d &R);
bool Vector2SkewSymmetric(Eigen::Vector3d w, Eigen::Matrix3d &w_hat);
bool se32SE3(Eigen::Vector3d w, Eigen::Vector3d r, double angle, Eigen::Matrix3d &R, Eigen::Vector3d &t);
bool se32translation(Eigen::Vector3d w, Eigen::Vector3d v, Eigen::Matrix3d R, double angle, Eigen::Vector3d &t);
bool se32SE3Full(Eigen::Vector3d w, Eigen::Vector3d r, double angle, Eigen::Isometry3d &T);



#endif //SRC_CPP_ROBOTICS_MATH_H
