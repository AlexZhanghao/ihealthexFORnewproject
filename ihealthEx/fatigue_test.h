#pragma once
#include "control_card.h"
#include "data_acquisition.h"
#include "file_writer.h"
#include "FTWrapper.h"
#include"sensordata_processing.h"
#include"boundarydetection.h"

#define CCHART_UPDATE (2050)

class FatigueTest {
public:
	FatigueTest();
	~FatigueTest();

	void Initial(HWND hWnd);
	bool IsInitialed();

	void StartTest();
	void StartAbsoulteMove();
	void StartActiveMove();
	void AbsoluteMove();
	void ActMove();
	//国产六维力
	void StartMove();
	//ATI
	void timerAcquisit();
	//压力传感器采集
	void PressureSensorAcquisit();
	//力矩传感器采集
	void TorqueAcquisit();
	//力矩传感器运动
	void TorqueMove();
	void PositionReset();
	void StopMove();
	bool IsErrorHappened();
	void AcquisiteData();
	void SetZero();
	//将力矩由主动关节换算到所有关节
	void ActiveTorqueToAllTorque(double torque[2], double alltorque[5]);
	//等待线程停止
	void ExitTorqueThread();

	//输出力到txt文件
	void ExportForceData();
	//输出力矩传感器数据到txt
	void ExportTorqueData();
	//输出压力传感器计算得的力矩数据到txt
	void ExportMomentData();
	//输出六维力数据到txt
	void ExportSixDimensionData();

public:
	HWND m_hWnd;
	double elbow_angle_error[500] { 0 };
	double shoulder_angle_error[500] { 0 };
	double elbow_angle_curve[500] { 0 };
	double shoulder_angle_curve[500] { 0 };
	double pull_force_curve1[500] { 0 };
	double pull_force_curve2[500] { 0 };
	double pull_force_curve3[500] { 0 };
	double pull_force_curve4[500] { 0 };
	double elbow_torque_curve[500] { 0 };
	double shoulder_torque_curve[500] { 0 };
	double x_axis[500] { 0 };

	bool m_stop = true;
	bool torque_collecting;
	bool torque_moving;

	double two_arm_offset[8];

public:
	
	double moments[6];
	double shoulder_torque; 
	double elbow_torque;
	double shoulder_tau;
	double elbow_tau;
	double shoulder_difference;
	double elbow_difference;
	double shoulder_fusion;
	double elbow_fusion;


private:
	void WriteDataToFile(int index);
	void UpdataDataArray();
	void UpdataDataArray2(double sensordata[8]);

	//将原始值进行坐标变换
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	//用来对压力传感器数据进行滤波
	void Trans2FilterForPressure(double TransData[4], double FiltedData[4]);
	void FiltedVolt2Vel(double FiltedData[6]);
	void MomentCalculation(double ForceVector[4],double vel[2]);
	//将六维力转移到关节空间
	void SixDimensionForceRotation(double sixdimensionforce[6]);
	//将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]);

private:
	bool is_initialed = false;
	bool in_test_move = false;
	bool is_testing = false;
	bool is_moving = false;

	FTWrapper *mFTWrapper;
	DataAcquisition *m_pDataAcquisition = nullptr;
	ControlCard *m_pControlCard = nullptr;
	FileWriter *m_pFileWriter = nullptr;
	sensorprocess m_psensorprocess;
	boundaryDetection m_boundarydetect;
	HANDLE test_thread = nullptr;
	HANDLE ATI_thread = nullptr;
	HANDLE acquisition_thread = nullptr;

	double m_shoulder_vel;
	double m_elbow_vel;

	double m_shoulder_moment;
	double m_elbow_moment;
	


	I32 option = 0x1000;//ptp运动模式控制
};