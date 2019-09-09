#pragma once
#include "control_card.h"
#include "data_acquisition.h"
#include "file_writer.h"
#include"sensordata_processing.h"
#include"boundarydetection.h"
#include<vector>

#define CCHART_UPDATE (2050)

using namespace std;

class FatigueTest {
public:
	FatigueTest();
	~FatigueTest();

	void Initial(HWND hWnd);
	bool IsInitialed();
	bool IsMoved();
	bool IsSixdimForceMoved();
	bool IsTested();

	void StartTest();
	void StartAbsoulteMove();
	void StartActiveMove();
	void AbsoluteMove();
	void PositionReset();
	void StopMove();
	bool IsErrorHappened();
	void SetZero();
	//�������������ؽڻ��㵽���йؽ�
	void ActiveTorqueToAllTorque(double torque[2], double alltorque[5]);
	void ExportDataToInterface(double data[8]);
	

	//�ȴ��߳�ֹͣ
	void ExitTorqueThread();
	void ExitPressureThread();
	void ExitSixdimForceThread();
	void ExitFatigueThread();

	//�������txt�ļ�
	void ExportForceData();
	//������ش��������ݵ�txt
	void ExportTorqueData();
	//���ѹ������������õ��������ݵ�txt
	void ExportMomentData();
	//�����ά�����ݵ�txt
	void ExportSixDimensionData();
	//���ƣ�Ͳ��Խ��
	void ExportFatigueData();

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
	double two_arm_offset[8];

	vector<double> fatigue_collection[6];

	bool m_stop = true;
	bool torque_collecting;
	bool torque_moving;
	bool is_fatigue;


public:
	double moments[6];
	double moment_to_interface[8];
	double shoulder_torque; 
	double elbow_torque;
	double shoulder_tau;
	double elbow_tau;
	double shoulder_difference;
	double elbow_difference;
	double shoulder_fusion;
	double elbow_fusion;
	double m_shoulder_moment;
	double m_elbow_moment;

private:
	void WriteDataToFile(int index);
	void UpdataDataArray();
	void UpdataDataArray2(double sensordata[8]);

	////����ά��ת�Ƶ��ؽڿռ�
	//void SixDimensionForceRotation(double sixdimensionforce[6]);

private:
	bool is_initialed = false;
	bool is_sixdimforce_move = false;
	bool is_testing = false;
	bool is_moving = false;

	DataAcquisition *m_pDataAcquisition = nullptr;
	ControlCard *m_pControlCard = nullptr;
	FileWriter *m_pFileWriter = nullptr;
	sensorprocess m_psensorprocess;
	boundaryDetection m_boundarydetect;
	HANDLE pressure_thread = nullptr;
	HANDLE sixdim_thread = nullptr;
	HANDLE torque_thread = nullptr;
	HANDLE fatiguetest_thread = nullptr;

	double m_shoulder_vel;
	double m_elbow_vel;

	static double six_dimforce[6];


	I32 option = 0x1000;//ptp�˶�ģʽ����
};