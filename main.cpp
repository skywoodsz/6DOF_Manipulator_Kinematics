#include <iostream>
#include "kuka_control.h"
#include "robotics_math.h"
using namespace Eigen;
void subproblem3(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double delta, vector<double>& theta)
{
    Vector3d u, v, u_, v_;
    u = p - r;
    v = q - r;

    double tmp = w.transpose() * (p - q);
    double delta_2 = delta * delta - tmp * tmp;

//    u_ = u - w.transpose() * u * w;
//    v_ = v - w.transpose() * v * w;

    u_ = u - w * w.transpose() * u;
    v_ = v - w * w.transpose() * v;

    double th0 = atan2(w.transpose() * u_.cross(v_), u_.transpose() * v_);

    double u_n2 = u_.dot(u_);
    double v_n2 = v_.dot(v_);

    double th_ = (u_n2 + v_n2 - delta_2) / sqrt(4 * u_n2 * v_n2);

    theta.emplace_back(180 / M_PI * (th0 - acos(th_)));
    theta.emplace_back(180 / M_PI * (th0 + acos(th_)));

}
/*
 * Input: ZYZ Eular angle
 *
 */
int main() {
    KUKA_CONTROL kuka_control;
//    Eigen::Vector3d w(0, 0, 1);
//    Eigen::Vector3d r(0, 2, 0);
//    double angle = 45.0 / 180.0 * M_PI;
//    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//    se32SE3Full(w, r, angle, T);
//    std::cout<<"T: "<<T.matrix()<<std::endl;
//    std::cout<<"angle: "<<angle<<std::endl;


    Vector6D th, th1, pos;
//    th << 29.27, -2.795, 89.227, -0.691,92.67, 183.769;
//    th = th * M_PI / 180;
//    kuka_control.ForwardKinematics(th, pos);

//    pos << 0.415746, -0.219622, 0.920636, -29.489 * M_PI / 180, 178.867 * M_PI / 180, -33.751 * M_PI / 180;
//    pos << 0.555893, -0.054405, 0.920636, -8.288 * M_PI / 180, 178.867 * M_PI / 180, -33.751 * M_PI / 180;
    pos << 0.374286, -0.208621, 0.884491, -8.287 * M_PI / 180, 178.867 * M_PI / 180, -33.751 * M_PI / 180;
    kuka_control.InverseKinematics(pos, th1);
    std::cout<<"th: "<<th1 * 180 / M_PI<<std::endl;

//    Vector6D pos, th;
//        pos << 0, 0, 1.475, 0, 0, -180 * M_PI / 180;
//    pos << 0.415746, -0.219622, 0.920636, -29.489 * M_PI / 180, 178.867 * M_PI / 180, -33.751 * M_PI / 180;
//    kuka_control.InverseKinematics(pos, th);
//    std::cout<<th * 180 / M_PI<<std::endl;
//    Eigen::Vector3d w(-1,0,0);
//    Eigen::Vector3d r(0, 0, 1);
//    double angle = 0;
//    Eigen::Matrix3d R;
//    Eigen::Vector3d t;
//    se32SE3(w, r, angle, R, t);
//    Eigen::Vector3d p(1, 0, 1);
//    Eigen::Vector3d q(1, -1, 0);
//    Eigen::Vector3d w(1,0,0);
//    Eigen::Vector3d r(0, 0, 0);
//    double delta = sqrt(0);
//    double th;
//    kuka_control.PKSubProblem3(p, q, r, w, delta, th);
//    std::cout<<"th: "<<th<<std::endl;
//    vector<double> theta;
//    subproblem3(r, p, q, w, delta, theta);
    return 0;
}

//    Eigen::Vector3d p(0, 1, 1);
//    Eigen::Vector3d q(1, 1, 0);
//    Eigen::Vector3d w(0,1,0);
//    Eigen::Vector3d r(0, 0, 0);
//    double th;
//    kuka_control.PKSubProblem1(p, q, w, r, th);
//    std::cout<<"th: "<<th<<std::endl;

//    Eigen::Vector3d p(0, -1, 1);
//    Eigen::Vector3d q(1, 1, 0);
//    Eigen::Vector3d w2(0,0,1);
//    Eigen::Vector3d w1(0,1,0);
//    Eigen::Vector3d r(0, 0, 0);
//    double th1, th2;
//    kuka_control.PKSubProblem2(p, q, r, w1, w2, th1, th2);
//    std::cout<<"th1: "<<th1 * 180 / M_PI<<std::endl;
//    std::cout<<"th2: "<<th2 * 180 / M_PI<<std::endl;
//
//    Eigen::Matrix3d R1, R2;
//    AxisAngle2Rorarion(w1, th1, R1);
//    AxisAngle2Rorarion(w2, th2, R2);

//    std::cout<<"V: "<<R2 * R1 * (p - r)<<std::endl;

//    Eigen::Vector3d p(1, 0, 1);
//    Eigen::Vector3d q(1, -1, 0);
//    Eigen::Vector3d w(1,0,0);
//    Eigen::Vector3d r(0, 0, 0);
//    double delta = sqrt(0);
//    double th;
//    kuka_control.PKSubProblem3(p, q, r, w, delta, th);
//    std::cout<<"th: "<<th<<std::endl;

//    Eigen::Vector3d w(0, 0, 1);
//    Eigen::Matrix3d w_hat;
//    Vector2SkewSymmetric(w, w_hat);
//    std::cout<<"w_hat: "<<w_hat<<std::endl;

//    Eigen::Vector3d w1(0, 0, 1), w2(0, 1, 0), u(0, -1, 1);
//    double angle = -180 * M_PI / 180;
//    Eigen::Matrix3d R1, R2;
//    AxisAngle2Rorarion(w1, 90 * M_PI / 180, R1);
//    AxisAngle2Rorarion(w2, 90 * M_PI / 180, R2);
//
//    std::cout<<"V: "<<R2 * R1 * u<<std::endl;
