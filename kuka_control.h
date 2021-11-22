//
// Created by skywoodsz on 2021/11/21.
//

#ifndef SRC_CPP_KUKA_CONTROL_H
#define SRC_CPP_KUKA_CONTROL_H
#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include "robotics_math.h"

#define Link_1       0.491
#define Link_2       0.450
#define Link_3      0.450
#define Link_4      0.084

#define JOINT1_MIN       - 170 / 180 * M_PI
#define JOINT1_MAX        170 / 180 * M_PI

#define JOINT2_MIN       - 120 / 180 * M_PI
#define JOINT2_MAX        120 / 180 * M_PI

#define JOINT3_MIN       - 228 / 180 * M_PI
#define JOINT3_MAX        48 / 180 * M_PI

#define JOINT4_MIN       - 170 / 180 * M_PI
#define JOINT4_MAX        170 / 180 * M_PI

#define JOINT5_MIN       - 120 / 180 * M_PI
#define JOINT5_MAX        120 / 180 * M_PI

typedef Eigen::Matrix<double, 6, 1> Vector6D;

using namespace std;

class KUKA_CONTROL {
public:
    KUKA_CONTROL();
    ~KUKA_CONTROL();

    bool ForwardKinematics(Vector6D th, Vector6D &pos);
    bool InverseKinematics(Vector6D pos, Vector6D &th);
    bool PKSubProblem1(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d w, Eigen::Vector3d r, double &th); // p, q, w, r
    bool PKSubProblem2(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r, Eigen::Vector3d w1, Eigen::Vector3d w2, double &th1, double &th2); //p, q, r, w1, w2, th1, th2
    bool PKSubProblem3(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r, Eigen::Vector3d w, double delta, double &th); //p, q, r, w, delta, th

public:
    Eigen::Isometry3d gst_0, gd, gst; // 位姿
    Eigen::Vector3d pw, p2, p1, p6, p7; // IK结算点
    Eigen::Vector3d w1, w2, w3, w4, w5, w6; // 旋转轴
};


#endif //SRC_CPP_KUKA_CONTROL_H
