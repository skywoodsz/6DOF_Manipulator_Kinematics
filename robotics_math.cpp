//
// Created by skywoodsz on 2021/11/22.
//
#include "robotics_math.h"

bool Vector2SkewSymmetric(Eigen::Vector3d w, Eigen::Matrix3d &w_hat)
{
    w_hat<<0, -w(2), w(1),
    w(2), 0, -w(0),
    -w(1), w(0), 0;
    return true;
}

bool AxisAngle2Rorarion(Eigen::Vector3d w, double angle, Eigen::Matrix3d &R)
{
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d w_hat;

    Vector2SkewSymmetric(w, w_hat);
//    std::cout<<"w_hat:"<<w_hat<<std::endl;

    R = I_33 + w_hat * sin(angle) + w_hat * w_hat * (1 - cos(angle));
    return true;
}

bool se32translation(Eigen::Vector3d w, Eigen::Vector3d v, Eigen::Matrix3d R, double angle, Eigen::Vector3d &t)
{
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d w_hat;
    Vector2SkewSymmetric(w, w_hat);

    t = ((I_33 - R) * w_hat + w * w.transpose() * angle) * v;
    return true;
}

bool se32SE3(Eigen::Vector3d w, Eigen::Vector3d r, double angle, Eigen::Matrix3d &R, Eigen::Vector3d &t)
{
    // v
    Eigen::Vector3d v;
    v = - w.cross(r);
//    std::cout<<"v:"<<v<<std::endl;

    // R
    AxisAngle2Rorarion(w, angle, R);

    //t
    se32translation(w, v, R, angle, t);
    return true;
}

bool se32SE3Full(Eigen::Vector3d w, Eigen::Vector3d r, double angle, Eigen::Isometry3d &T)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    se32SE3(w, r, angle, R, t);
    T.rotate(R);
    T.pretranslate(t);

//    std::cout<<"R: "<<R<<std::endl;
//    std::cout<<"t: "<<t<<std::endl;
    return true;
}