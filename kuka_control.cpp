//
// Created by skywoodsz on 2021/11/21.
//

#include "kuka_control.h"

KUKA_CONTROL::KUKA_CONTROL()
{
    // 初始位姿
    Eigen::AngleAxisd rotation_vector_0(M_PI, Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d t0(0, 0, Link_1 + Link_2 + Link_3 + Link_4);

    gst_0 = Eigen::Isometry3d::Identity();
    gst_0.rotate(rotation_vector_0);
    gst_0.pretranslate(t0);

    gd = Eigen::Isometry3d::Identity();
    gst = Eigen::Isometry3d::Identity();

    // IK结算点
    pw << 0, 0, Link_1 + Link_2 + Link_3;
    p2 << 0, 0, Link_1 + Link_2;
    p1 << 0, 0, Link_1;
    p6 << 0, 0, Link_1 + Link_2 + Link_3 + Link_4;
    p7 << 1, 0, Link_1 + Link_2 + Link_3 + Link_4;

    // 旋转轴
    w1 << 0, 0, 1;
    w2 << 0, 1, 0;
    w3 << 0, 1, 0;
    w4 << 0, 0, 1;
    w5 << 0, 1, 0;
    w6 << 0, 0, 1;
}
KUKA_CONTROL::~KUKA_CONTROL() {}

bool KUKA_CONTROL::ForwardKinematics(Vector6D th, Vector6D &pos)
{
    Eigen::Isometry3d gst1, gst2, gst3, gst4, gst5, gst6;
    gst1 = Eigen::Isometry3d::Identity();
    gst2 = Eigen::Isometry3d::Identity();
    gst3 = Eigen::Isometry3d::Identity();
    gst4 = Eigen::Isometry3d::Identity();
    gst5 = Eigen::Isometry3d::Identity();
    gst6 = Eigen::Isometry3d::Identity();

    se32SE3Full(w6, pw, th(5), gst6);
    se32SE3Full(w5, pw, th(4), gst5);
    se32SE3Full(w4, pw, th(3), gst4);
    se32SE3Full(w3, p2, th(2), gst3);
    se32SE3Full(w2, p1, th(1), gst2);
    se32SE3Full(w1, p1, th(0), gst1);

    gst = gst1 * gst2 * gst3 * gst4 * gst5 * gst6 * gst_0;

//    std::cout<<"th: "<<th<<std::endl;
//    std::cout<<"w6: "<<w6<<std::endl;
//    std::cout<<"pw: "<<pw<<std::endl;

//    std::cout<<"gst: "<<std::endl;
//    std::cout<<gst.matrix()<<std::endl;

//    std::cout<<"gst_0: "<<std::endl;
//    std::cout<<gst_0.matrix()<<std::endl;

//    std::cout<<"gst6: "<<std::endl;
//    std::cout<<gst6.matrix()<<std::endl;
}

/*
 * Kuka manipulator IK
 * Input: 期望位姿 se3 pos(x, y, z, roll, pitch, yaw)
 * Output: 六轴转动角度值 th
 */
bool KUKA_CONTROL::InverseKinematics(Vector6D pos, Vector6D &th)
{
    double th1, th2, th3, th4, th5, th6;
    Eigen::Isometry3d g1, g2, g3;
    Eigen::Vector3d p, q, r, vec3d_temp;
    double delta;

    // 0. gd
    Eigen::AngleAxisd r_vecd_yaw(pos(3), Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd r_vecd_pitch(pos(4), Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd r_vecd_roll(pos(5), Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d td(pos(0), pos(1), pos(2));

    gd = Eigen::Isometry3d::Identity();
    gd.rotate(r_vecd_yaw);
    gd.rotate(r_vecd_pitch);
    gd.rotate(r_vecd_roll);
    gd.pretranslate(td);
//    std::cout<<"gd:"<<std::endl;
//    std::cout<<gd.matrix()<<std::endl;

    // debug
//    gd = gst;

    // 1. g1
    g1 = gd * gst_0.inverse();
//    std::cout<<"g1"<<std::endl;
//    std::cout<<g1.matrix()<<std::endl;

    // 2. pw --> th3
    p = pw; q = p1; r << 0, 0, Link_1 + Link_2;
    vec3d_temp = g1 * pw - p1;
    delta = vec3d_temp.norm();
    PKSubProblem3(p, q, r, w3, delta, th3);

    // 3. pw --> th1, th2
    Eigen::Isometry3d g_w3 = Eigen::Isometry3d::Identity();
    se32SE3Full(w3, p2, th3, g_w3);
//    std::cout<<"g_w3 "<<g_w3.matrix()<<std::endl;

    p = g_w3 * pw;
    q = g1 * pw;
    r = p1;
    PKSubProblem2(p, q, r, w1, w2, th1, th2);

    // 4. g2, p6 --> th4, th5
    Eigen::Isometry3d g_temp;
    Eigen::Isometry3d g_w2 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d g_w1 = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R2, R1;
    Eigen::Vector3d t2, t1;

    se32SE3(w2, r, th2, R2, t2);
    g_w2.rotate(R2); g_w2.pretranslate(t2);

    se32SE3(w1, r, th1, R1, t1);
    g_w1.rotate(R1); g_w1.pretranslate(t1);

    g_temp = g_w1 * g_w2 * g_w3;
    g2 = g_temp.inverse() * g1;

    p = p6; q = g2 * p6; r = pw;
    PKSubProblem2(p, q, r, w4, w5, th4, th5);

    // 5. g3, p7 --> th1
    Eigen::Isometry3d g_w4 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d g_w5 = Eigen::Isometry3d::Identity();

    Eigen::Matrix3d R4, R5;
    Eigen::Vector3d t4, t5;

    se32SE3(w4, r, th4, R4, t4);
    g_w4.rotate(R4); g_w4.pretranslate(t4);

    se32SE3(w5, r, th5, R5, t5);
    g_w5.rotate(R5); g_w5.pretranslate(t5);

    g_temp = g_w4 * g_w5;
    g3 = g_temp.inverse() * g2;
    p = p7; q = g3 * p7; r = p6;
    PKSubProblem1(p, q, w6, r, th6);

    th << th1, th2, th3, th4, th5, th6 + 2 * M_PI;

    return true;
}

/*
 * PK子问题1
 * Input: 两点 p, q; 旋转轴 w; 轴上一点 r
 * Output: 旋转角度 th
 */
bool KUKA_CONTROL::PKSubProblem1(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d w, Eigen::Vector3d r, double &th)
{
    Eigen::Vector3d u, v, u_temp, v_temp;

    u = p - r;
    v = q - r;

    u_temp = u - w * w.transpose() * u;
    v_temp = v - w * w.transpose() * v;

    th = atan2(w.transpose() * (u_temp.cross(v_temp)), u_temp.dot(v_temp));

    return true;
}

/*
 * PK子问题2
 * Input: 两点 p, q; 两旋转轴 w1, w2; 轴上一点 r
 * Notes: 先转为2轴和角度2, 后转为1轴与角度1
 * Output: 旋转角度 th1, th2
 */
bool KUKA_CONTROL::PKSubProblem2(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r, Eigen::Vector3d w1,
                                 Eigen::Vector3d w2, double &th1, double &th2)
{
    Eigen::Vector3d u, v, z, c;
    double alpha, beta, gama;

    u = p - r;
    v = q - r;

    // 求解中间向量z
    double temp1, temp2, temp3;
    Eigen::Vector3d vec_temp;
    temp1 = (w1.dot(w2)) * w2.dot(u) - w1.dot(v);
    temp2 = pow((w1.dot(w2)), 2) - 1;

    alpha = temp1 / temp2;
    temp1 = (w1.dot(w2)) * w1.dot(v) - w2.dot(u);
    beta = temp1 / temp2;

    temp1 = u.norm() * u.norm() - alpha * alpha - beta * beta - 2 * alpha * beta * w1.dot(w2);
    vec_temp = w1.cross(w2);
    temp2 = vec_temp.norm() * vec_temp.norm();
    temp3 = sqrt(temp1 / temp2);
    // 多解情况取负值
    if(temp3 < 0)
    {
        gama = temp3;
    }
    else
    {
        gama = - temp3;
    }

    z = alpha * w1 + beta * w2 + gama * (w1.cross(w2));
    c = z + r;

//    std::cout<<"alpha: "<<alpha<<std::endl;
//    std::cout<<"beta: "<<beta<<std::endl;
//    std::cout<<"gama: "<<gama<<std::endl;
//    std::cout<<"z: "<<z<<std::endl;
    // 转换为subproblem1
    PKSubProblem1(c, q, w1, r, th1);
    PKSubProblem1(p, c, w2, r, th2);

    return true;
}
/*
 * PK 子问题3
 * Input: 两点 p, q; 旋转轴 w; 轴上一点 r, 平移距离 delta
 * Output: 旋转角度 th
 */
bool KUKA_CONTROL::PKSubProblem3(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r, Eigen::Vector3d w,
                                 double delta, double &th)
{
    Eigen::Vector3d u, v,  u_temp, v_temp;
    double delta_temp, temp, th0;

    u = p - r;
    v = q - r;

    u_temp = u - w * w.transpose() * u;
    v_temp = v - w * w.transpose() * v;

    temp = w.dot(p - q);
    delta_temp = sqrt(delta * delta - temp * temp);

    temp = (u_temp.dot(u_temp) + v_temp.dot(v_temp) - delta_temp * delta_temp) /
            (2 * u_temp.norm() * v_temp.norm());

    th0 = atan2(w.transpose() * (u_temp.cross(v_temp)), u_temp.dot(v_temp));
    th = th0 - acos(temp); // +

//    std::cout<<"th0: "<<th0<<std::endl;

    return true;
}

