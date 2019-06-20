#pragma once
#include "control_card.h"
#include "data_acquisition.h"
#include "file_writer.h"
#include "FTWrapper.h"

#define CCHART_UPDATE (2050)

class FatigueTest {
public:
	FatigueTest() = default;

	void Initial(HWND hWnd);
	bool IsInitialed();

	void StartTest();
	//国产六维力
	void StartMove();
	//ATI
	void timerAcquisit();
	//压力传感器
	void PressureSensorAcquisit();
	void PositionReset();
	void StopMove();
	void AbsoluteMove();
	bool IsErrorHappened();
	void AcquisiteData();

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

	double two_arm_offset[8];

public:
	
	double moments[6];
	double shoulder_moment;
	double elbow_moment;
	double shoulder_tau;
	double elbow_tau;
	double shoulder_difference;
	double elbow_difference;


private:
	void WriteDataToFile(int index);
	void UpdataDataArray();
	void UpdataDataArray(double buf[6]);
	void UpdataDataArray2(double sensordata[8]);

	//将原始值进行坐标变换
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	void FiltedVolt2Vel(double FiltedData[6]);

private:
	bool is_initialed = false;
	bool in_test_move = false;

	FTWrapper mFTWrapper;
	DataAcquisition *m_pDataAcquisition = nullptr;
	ControlCard *m_pControlCard = nullptr;
	FileWriter *m_pFileWriter = nullptr;
	HANDLE test_thread = nullptr;
	HANDLE acquisition_thread = nullptr;

	double m_shoulder_vel;
	double m_elbow_vel;


};