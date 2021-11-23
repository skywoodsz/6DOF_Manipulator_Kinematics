#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include <algorithm>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

#define PI M_PI

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

	mSampleTime = 0.001;
	//mVel = 0;
	mAcc = 0;
	//mDec = 0;
	Td = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double acc, double td)
{
	//mVel = vel;
	mAcc = acc;
	Td = td;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(Vector6D StartTh, Vector6D EndTh)
{

    mJointAngleBegin[0] = StartTh[0];//angle1;
    mJointAngleBegin[1] = StartTh[1];//angle2;
    mJointAngleBegin[2] = StartTh[2];//angle3;
    mJointAngleBegin[3] = StartTh[3];//angle4;
    mJointAngleBegin[4] = StartTh[4];//angle5;
    mJointAngleBegin[5] = StartTh[5];//angle6;

    mJointAngleEnd[0] = EndTh[0];//angle1;
    mJointAngleEnd[1] = EndTh[1];//angle2;
    mJointAngleEnd[2] = EndTh[2];//angle3;
    mJointAngleEnd[3] = EndTh[3];//angle4;
    mJointAngleEnd[4] = EndTh[4];//angle5;
    mJointAngleEnd[5] = EndTh[5];//angle6;
}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::getTime(const double& bAngle, const double& eAngle, double& t1, double& t2)
{
	//double t_acc, minAcc;
	double minAcc;
	minAcc = 4 * abs(eAngle - bAngle) / Td / Td;

	if (mAcc < minAcc) {
        mAcc = minAcc;
		t1 = Td / 2;
		t2 = Td - t1;
	}
	else {
		t1 = Td / 2 - sqrt(mAcc*mAcc*Td*Td - 4 * mAcc*abs(eAngle - bAngle)) / 2 / mAcc;
		t2 = Td - t1;
	}

	// t_acc = mVel / mAcc;

	// double flag = (eAngle - bAngle) >= 0 ? 1.0 : -1.0;
	// double acc = flag * mAcc;
	// double dec = flag * mDec;
	// double vm = flag * mVel;
	// double t_rest = (eAngle - bAngle - 0.5 * acc * t_acc * t_acc - 0.5 * dec * t_dec * t_dec) / vm;

	// if (t_rest > 0) {
		// te = t_acc + t_dec + t_rest;
		// t1 = t_acc;
		// t2 = t1 + t_rest;
	// }
	// else {
		// t1 = sqrt((eAngle - bAngle) / (acc * (acc + dec) / (2.0 * dec)));
		// te = t1 + sqrt((eAngle - bAngle) / (dec * (acc + dec) / (2.0 * acc)));
		// t2 = t1;
	// }
}

void CHLMotionPlan::getTime(const double& bAngle, const double& eAngle, const double& Acc,
	double& t1, double& t2)
{
	double minAcc;
	minAcc = 4 * abs(eAngle - bAngle) / Td / Td;

	if (Acc < minAcc) {
		t1 = Td / 2;
		t2 = Td - t1;
	}
	else {
		t1 = Td / 2 - sqrt(Acc*Acc*Td*Td - 4 * Acc*abs(eAngle - bAngle)) / 2 / Acc;
		t2 = Td - t1;
	}

	// double t_acc, t_dec;

	// t_acc = Vel / Acc;
	// t_dec = Vel / Dec;

	// double flag = (eAngle - bAngle) >= 0 ? 1.0 : -1.0;
	// double acc = flag * Acc;
	// double dec = flag * Dec;
	// double vm = flag * Vel;
	// double t_rest = (eAngle - bAngle - 0.5 * acc * t_acc * t_acc - 0.5 * dec * t_dec * t_dec) / vm;

	// if (t_rest > 0) {
		// te = t_acc + t_dec + t_rest;
		// t1 = t_acc;
		// t2 = t1 + t_rest;
	// }
	// else {
		// t1 = sqrt((eAngle - bAngle) / (acc * (acc + dec) / (2.0 * dec)));
		// te = t1 + sqrt((eAngle - bAngle) / (dec * (acc + dec) / (2.0 * acc)));
		// t2 = t1;
	// }
}
inline void CHLMotionPlan::LFPB(const double& th0, const double& the, const double& t1, const double& t2, double t, double& th)
{
	double flag = (the - th0) >= 0 ? 1.0 : -1.0;
	double acc = flag * mAcc;

	if (t >= 0 && t < t1)
	{
		th = th0 + 0.5 * acc * t * t;
	}
	else if (t >= t1 && t < t2)		// 当出现没有匀速段时， 这个判断自动无效
	{
		th = th0 + acc * t1 *(t - t1 / 2);
	}
	else if (t >= t2 && t <= Td)
	{
		th = the - 0.5 * acc * (Td - t) * (Td - t);
	}
	else {
		cout << "LFPB error" << endl;
	}

	// double flag = (the - th0) >= 0 ? 1.0 : -1.0;
	// double acc = flag * mAcc;
	// double dec = flag * mDec;
	// double v = acc * t1;
	// if (t >= 0 && t < t1)
	// {
		// th = th0 + 0.5 * acc * t * t;
	// }
	// else if (t >= t1 && t < t2)		// 当出现没有匀速段时， 这个判断自动无效
	// {
		// th = th0 + v * (t - t1 / 2);
	// }
	// else if (t >= t2 && t <= te)
	// {
		// th = the - 0.5 * dec * (te - t) * (te - t);
	// }
	// else {
		// cout << "LFPB error" << endl;
	// }
}

inline void CHLMotionPlan::LFPB(const double& th0, const double& the, const double& t1, const double& t2,
	const double& Acc, double t, double& th)
{
	double flag = (the - th0) >= 0 ? 1.0 : -1.0;
	double acc = flag * Acc;

	//if (t >= 0 && t < t1)
	//{
	//	th = th0 + 0.5 * acc * t * t;
	//}
	//else if (t >= t1 && t < t2)		// 当出现没有匀速段时， 这个判断自动无效
	//{
	//	th = th0 + acc * t1 *(t - t1 / 2);
	//}
	//else if (t >= t2 && t <= te)
	//{
	//	th = the - 0.5 * acc * (Td - t) * (Td - t);
	//}
	//else {
	//	cout << "LFPB error" << endl;
	//}

	// double flag = (the - th0) >= 0 ? 1.0 : -1.0;
	// double acc = flag * Acc;
	// double dec = flag * Dec;
	// double v = acc * t1;
	// if (t >= 0 && t < t1)
	// {
		// th = th0 + 0.5 * acc * t * t;
	// }
	// else if (t >= t1 && t < t2)		// 当出现没有匀速段时， 这个判断自动无效
	// {
		// th = th0 + v * (t - t1 / 2);
	// }
	// else if (t >= t2 && t <= te)
	// {
		// th = the - 0.5 * dec * (te - t) * (te - t);
	// }
	// else {
		// cout << "LFPB error" << endl;
	// }
}

void CHLMotionPlan::GetPlanPoints(ofstream &outfile)
{

//	outfile.open("data.txt");
//	outfile << mJointAngleBegin[0] << "  "
//		<< mJointAngleBegin[1] << "  "
//		<< mJointAngleBegin[2] << "  "
//		<< mJointAngleBegin[3] << "  "
//		<< mJointAngleBegin[4] << "  "
//		<< mJointAngleBegin[5] << "  ";
//	outfile << endl;
	//To Do
	vector<vector<double>> theta;
	theta.resize(6);
	// int max_te_id = -1;  // 顺便找到最大te的索引
	// double max_te = 0;
	for (int i = 0; i < 6; i++)
	{
		double t1, t2;
		getTime(mJointAngleBegin[i], mJointAngleEnd[i], t1, t2);
		// if (te > max_te) {
			// max_te = te;
			// max_te_id = i;
		// }
		double cur_time = 0;
		double th;
		while (cur_time <= Td) {
			LFPB(mJointAngleBegin[i], mJointAngleEnd[i], t1, t2, cur_time, th);
			theta[i].emplace_back(th);
			cur_time += mSampleTime;
		}
		theta[i].emplace_back(mJointAngleEnd[i]);
	}

	int num_data = theta[0].size();

	// 对齐theta
	// int num_data = theta[max_te_id].size();
	// for (int i = 0; i < 4; i++) {
		// if (i == max_te_id)  continue;

		// int lack = num_data - theta[i].size();
		// vector<double> tmp(lack, theta[i].back());
		// theta[i].insert(theta[i].end(), tmp.begin(), tmp.end());
	// }

	for (int i = 0; i < num_data; i++)
	{
		outfile << theta[0][i] << "  "
			<< theta[1][i] << "  "
			<< theta[2][i] << "  "
			<< theta[3][i] << "  "
			<< theta[4][i] << "  "
			<< theta[5][i] << "  ";
		outfile << endl;
	}

//	outfile.close();
	//End of To Do

}

void CHLMotionPlan::GetPlanPoints_line(PosStruct Start, PosStruct End, string filename)
{
	//完成代码

	vector<vector<double>> cart;
	cart.resize(6);

	vector<double> pstart = { Start.x, Start.y, Start.z, Start.yaw, Start.pitch, Start.roll };
	vector<double> pend = { End.x, End.y, End.z, End.yaw, End.pitch, End.roll };
	Vector3d pos_start;
	pos_start << Start.x, Start.y, Start.z;

	Vector3d pos_end;
	pos_end << End.x, End.y, End.z;

	Vector3d pos_dir = (pos_end - pos_start);
	pos_dir = pos_dir / pos_dir.norm();
	for (int i = 0; i < 3; i++)	pos_dir(i) = fabs(pos_dir(i));

	//vector<double> pos_acc = { pos_dir(0) * mAcc, pos_dir(1) * mAcc, pos_dir(2) * mAcc };
	//vector<double> pos_dec = { pos_dir(0) * mDec, pos_dir(1) * mDec, pos_dir(2) * mDec };
	//vector<double> pos_vel = { pos_dir(0) * mVel, pos_dir(1) * mVel, pos_dir(2) * mVel };
	////对yaw,pitch,roll三个方向的插补
	//for (int i = 3; i < 6; i++)
	//{
	//	double t1, t2, te;
	//	getTime(pstart[i], pend[i], t1, t2, te);
	//	double cur_time = 0;
	//	double th;
	//	cart[i].emplace_back(pstart[i]);
	//	while (cur_time <= te)
	//	{
	//		LFPB(pstart[i], pend[i], t1, t2, te, cur_time, th);
	//		cart[i].emplace_back(th);
	//		cur_time += mSampleTime;
	//	}
	//	cart[i].emplace_back(pend[i]);
	//}
	////x,y,z三个方向的直线插补
	//for (int i = 0; i < 3; i++)
	//{
	//	double t1, t2, te;
	//	getTime(pstart[i], pend[i], pos_acc[i], pos_dec[i], pos_vel[i], t1, t2, te);
	//	double cur_time = 0;
	//	double th;
	//	cart[i].emplace_back(pstart[i]);
	//	while (cur_time <= te) {
	//		LFPB(pstart[i], pend[i], t1, t2, te, pos_acc[i], pos_dec[i], cur_time, th);
	//		cart[i].emplace_back(th);
	//		cur_time += mSampleTime;
	//	}
	//	cart[i].emplace_back(pend[i]);
	//}

	// 对齐数据
	int num_data = 0;
	int max_id = -1;
	for (int i = 0; i < 6; i++)
	{
		if (cart[i].size() > num_data) {
			num_data = cart[i].size();
			max_id = i;
		}
	}

	for (int i = 0; i < 6; i++) {
		if (i == max_id)  continue;
		int lack = num_data - cart[i].size();
		if (lack > 0) {
			vector<double> tmp(lack, cart[i].back());
			cart[i].insert(cart[i].end(), tmp.begin(), tmp.end());
		}
	}

	// output
	ofstream outfile2;
	outfile2.open(filename);

	for (int i = 0; i < num_data; i++)
	{
		//SetRobotEndPos(cart[0][i], cart[1][i], cart[2][i], cart[3][i], cart[4][i], cart[5][i]);
		//Matrix<double, 4, 1> j;
		//GetJointAngles(j(0), j(1), j(2), j(3));

		//outfile2 << j(0) << "  "
		//	<< j(1) << "  "
		//	<< j(2) << "  "
		//	<< j(3) << "  ";
		//outfile2 << endl;
		outfile2 << cart[0][i] << " "
			<< cart[1][i] << " "
			<< cart[2][i] << " "
			<< cart[3][i] << " "
			<< cart[4][i] << " "
			<< cart[5][i] << " ";
		outfile2 << endl;
	}
	outfile2.close();

}