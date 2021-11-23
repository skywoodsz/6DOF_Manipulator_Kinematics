//#pragma once
//#include <vector>
//using namespace std;
//
//struct PosStruct
//{
//	double x;				// x坐标，单位mm
//	double y;				// y坐标，单位mm
//	double z;				// z坐标，单位mm
//	double yaw;				// yaw坐标，单位度
//	double pitch;			// pitch坐标，单位度
//	double roll;			// roll坐标，单位度
//	bool config;
//};
//
//class CHLMotionPlan
//{
//private:
//	double mJointAngleBegin[4];					//起始点位的关节角度,单位度
//	double mJointAngleEnd[4];					//结束点位的关节角度，单位度
//	double mStartMatrixData[16];				//起始点位的转换矩阵数组
//	double mEndMatrixData[16];					//结束点位的转换矩阵数组
//	double mSampleTime;							//采样点位，单位S
//	double mVel;								//速度，单位m/s
//	double mAcc;								//加速度，单位m/s/s
//	double mDec;								//减速度，单位m / s / s
//	bool mConfig[3];							//机器人姿态
//
//public:
//	CHLMotionPlan();
//	virtual ~CHLMotionPlan();
//
//	void SetSampleTime(double sampleTime);		//设置采样时间
//	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		//输入起始点位和结束点位的笛卡尔坐标
//	void SetProfile(double vel, double acc, double dec);			//设置运动参数，速度、加速度和减速度
//	void GetPlanPoints();											//关节空间梯形速度规划
//	void GetPlanPoints_line();       								//笛卡尔空间直线轨迹梯形速度规划
//	double Gettime(double dtheta);
//	void getTime(const double& bAngle, const double& eAngle, double& t1, double& t2, double& te);
//	void Getdisperse(vector<double> &theta, int num, double theta0, double thetaf, double tf, double tb);
//	void getTime(const double& bAngle, const double& eAngle, const double& Acc, const double& Dec,
//		const double& Vel, double& t1, double& t2, double& te);
//	inline void LFPB(const double& th0, const double& the, const double& t1, const double& t2, const double& te, double t, double& th);
//	inline void LFPB(const double& th0, const double& the, const double& t1, const double& t2, const double& te,
//		const double& Acc, const double& Dec, double t, double& th);
//	void GetPlanPoints_line(PosStruct Start, PosStruct End, string filename);
//};


#pragma once
#include <vector>
#include "kuka_control.h"
using namespace std;

struct PosStruct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
	bool config;
};

class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	//double mVel;								//速度，单位m/s
	double mAcc;								//加速度，单位m/s/s
	double Td;                                  //运行时间
	//double mDec;								//减速度，单位m / s / s
	bool mConfig[3];							//机器人姿态

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);		//设置采样时间
	void SetPlanPoints(Vector6D StartTh, Vector6D EndTh);		//输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double acc, double td);			//设置运动参数，速度、加速度和减速度
	void GetPlanPoints(ofstream &outfile);											//关节空间梯形速度规划
	void GetPlanPoints_line();       								//笛卡尔空间直线轨迹梯形速度规划
	double Gettime(double dtheta);
	void getTime(const double& bAngle, const double& eAngle, double& t1, double& t2);
	void Getdisperse(vector<double> &theta, int num, double theta0, double thetaf, double tf, double tb);
	void getTime(const double& bAngle, const double& eAngle, const double& Acc,
		double& t1, double& t2);
	inline void LFPB(const double& th0, const double& the, const double& t1, const double& t2, double t, double& th);
	inline void LFPB(const double& th0, const double& the, const double& t1, const double& t2,
		const double& Acc, double t, double& th);
	void GetPlanPoints_line(PosStruct Start, PosStruct End, string filename);
};
