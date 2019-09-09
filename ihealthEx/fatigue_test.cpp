#include "fatigue_test.h"
#include"activemove_acquisition.h"
#include<vector>
#include <process.h>
#include <string>
#include <iostream>
#include<fstream>
#include "Matrix.h"

#define BOYDET_TIME 0.1

double Force_Fc = 0.3;
double Force_a = 0.3;
double Force_b = 1;
double shoulder_offset ;
double elbow_offset ;
double moment1[8] { 0.0 };
double torque_vel[8];
double pressure_data1;

double Ud_Arm = 0;//力控模式算出手臂的命令速度
double Ud_Shoul = 0;//力控模式算出肩部的命令速度

const double shoulder_moment_variance = 4.3769 / 10000;
const double elbow_moment_variance = 1.2576 / 10000;
const double shoulder_torque_variance = 1.4634 / 100000;
const double elbow_torque_variance = 2.2896 / 100000;
const double shoulder_pos = 40;
const double elbow_pos = 40;
const double sixdim = 0.015;
double sixdim_elbow_moment;
	
vector<double> force_collection[4];
vector<double> torque_collection[2];
vector<double> moment_collection[2];
vector<double> sixdimension_collection;

double FatigueTest::six_dimforce[6]{ 0 };

unsigned int __stdcall PressureThread(PVOID pParam);
unsigned int __stdcall SixdimThread(PVOID pParam);
unsigned int __stdcall TorqueThread(PVOID pParam);
unsigned int __stdcall FatigueTestThread(PVOID pParam);

FatigueTest::FatigueTest() {
	shoulder_torque = 0;
	elbow_torque = 0;
	shoulder_fusion = 0;
	elbow_fusion = 0;
	is_moving = false;
	torque_collecting = false;
	torque_moving = false;
	is_fatigue = false;
}

FatigueTest::~FatigueTest() {
	if (m_pControlCard != NULL) {
		delete m_pControlCard;
	}

	if (m_pDataAcquisition != NULL) {
		delete m_pDataAcquisition;
	}

	if (m_pFileWriter != NULL) {
		delete m_pFileWriter;
	}
}

void FatigueTest::Initial(HWND hWnd) {
	m_hWnd = hWnd;
	m_pControlCard = new ControlCard();
	m_pControlCard->Initial();
	m_boundarydetect.startBydetect();
	m_pDataAcquisition = new DataAcquisition();
	m_pFileWriter = new FileWriter();

	for (int i = 0; i < 500; ++i) {
		x_axis[i] = i;
	}
	is_initialed = true;
}

bool FatigueTest::IsInitialed() {
	return is_initialed;
}

bool FatigueTest::IsMoved() {
	return is_moving;
}

bool FatigueTest::IsSixdimForceMoved() {
	return is_sixdimforce_move;
}

bool FatigueTest::IsTested() {
	return is_testing;
}

void FatigueTest::StartTest() {

	is_testing = true;
	torque_collecting = true;
	is_sixdimforce_move = true;

	//压力传感器运动线程
	pressure_thread = (HANDLE)_beginthreadex(NULL, 0, PressureThread, this, 0, NULL);
	//六维力线程启动，这里要小心线程冲突
	sixdim_thread = (HANDLE)_beginthreadex(NULL, 0, SixdimThread, this, 0, NULL);
	//力矩传感器开启线程
	//torque_thread = (HANDLE)_beginthreadex(NULL, 0, TorqueThread, this, 0, NULL);
}

void FatigueTest::StartAbsoulteMove() {
	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);

	is_moving = true;
	is_fatigue = true;

	//打开疲劳测试线程
	fatiguetest_thread = (HANDLE)_beginthreadex(NULL, 0, FatigueTestThread, this, 0, NULL);

	AbsoluteMove();
}

void FatigueTest::StartActiveMove() {
	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);

	is_testing = true;
	is_moving = true;
	torque_moving = true;
	is_sixdimforce_move = true;
	torque_collecting = false;

	//主动运动线程
	pressure_thread = (HANDLE)_beginthreadex(NULL, 0, PressureThread, this, 0, NULL);
	//六维力线程启动，这里要小心线程冲突
	sixdim_thread = (HANDLE)_beginthreadex(NULL, 0, SixdimThread, this, 0, NULL);
	////力矩传感器开启线程
	//torque_thread = (HANDLE)_beginthreadex(NULL, 0, TorqueThread, this, 0, NULL);
}

void FatigueTest::AbsoluteMove() {
	if (!IsInitialed()) {
		return;
	}
	I32 Axis[2] = { SHOULDER_AXIS_ID,ELBOW_AXIS_ID };

	APS_ptp_v(Axis[0], option, shoulder_pos / VEL_TO_PALSE, 3 /  VEL_TO_PALSE, NULL);
	APS_ptp_v(Axis[1], option, elbow_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE, NULL);
	//APS_absolute_move(Axis[0], shoulder_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
	//APS_absolute_move(Axis[1], elbow_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
}

//void FatigueTest::SixDimensionForceRotation(double sixdimensionforce[6]) {
//	VectorXd sixdim(6);
//	Vector3d sixdim_to_coordinate3 = Vector3d(d3 - shouler_installationsite_to_coordinate4, sixdim_shoulder, 0);
//
//	for (int i = 0; i < 6; ++i) {
//		sixdim(i) = sixdimensionforce[i];			
//	}
//	
//	MatrixXd sixdim_transfer(6,6);
//	Matrix3d sixdim_rotation;
//	Matrix3d to_zero;
//	Matrix3d Sixdim_To_Coordinate3;
//
//	VectorToMatrix(sixdim_to_coordinate3, Sixdim_To_Coordinate3);
//	to_zero.setZero();
//
//	sixdim_rotation <<
//		1, 0, 0,
//		0, -1, 0,
//		0, 0, -1;
//
//	sixdim_transfer <<
//		sixdim_rotation, to_zero,
//		Sixdim_To_Coordinate3*sixdim_rotation, sixdim_rotation;
//
//	sixdim = sixdim_transfer*sixdim;
//	for (int i = 0; i < 6; ++i) {
//		sixdimensionforce[i] = sixdim(i);
//	}
//}

void FatigueTest::StopMove() {
	is_sixdimforce_move = false;
	is_testing = false;
	is_moving = false;
	torque_collecting = false;
	torque_moving = false;
	is_fatigue = false;
	//这里不放开离合的原因是为了防止中间位置松开离合导致手臂迅速下坠
	m_pControlCard->SetMotor(MOTOR_OFF);
	ExitTorqueThread();
}

void FatigueTest::PositionReset() {
	if (!IsInitialed()) {
		return;
	}
	m_pControlCard->PositionReset();
}

void FatigueTest::ExportDataToInterface(double data[8]) {
	UpdataDataArray2(data);
	PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);
}

//序号 肘部编码器 肩部编码器 拉力1 拉力2 拉力3 拉力4 肘部力矩 肩部力矩 "\n";
void FatigueTest::WriteDataToFile(int index) {
	m_pFileWriter->WriteString(std::to_string(index));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->elbow_position_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->shoulder_position_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->elbow_error_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->shoulder_error_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[0]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[1]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[2]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[3]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->torque_data[0]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->torque_data[1]));
	m_pFileWriter->WriteString("\n");
}

void FatigueTest::UpdataDataArray() {
	for (int i = 0; i < 499; i++) {
		elbow_angle_error[i] = 0;
		shoulder_angle_error[i] = 0;
		elbow_angle_curve[i] = 0;
		shoulder_angle_curve[i] = 0;
		pull_force_curve1[i] = 0;
		pull_force_curve2[i] = 0;
		pull_force_curve3[i] = 0;
		pull_force_curve4[i] = 0;
	}
}

void FatigueTest::UpdataDataArray2(double sensordata[8]) {
	for (int i = 0; i < 499; i++) {
		elbow_angle_error[i] = elbow_angle_error[i + 1];
		shoulder_angle_error[i] = shoulder_angle_error[i + 1];
		elbow_angle_curve[i] = elbow_angle_curve[i + 1];
		shoulder_angle_curve[i] = shoulder_angle_curve[i + 1];
		pull_force_curve1[i] = pull_force_curve1[i + 1];
		pull_force_curve2[i] = pull_force_curve2[i + 1];
		pull_force_curve3[i] = pull_force_curve3[i + 1];
		pull_force_curve4[i] = pull_force_curve4[i + 1];
	}
	elbow_angle_error[499] = sensordata[0];
	shoulder_angle_error[499] = sensordata[1];
	elbow_angle_curve[499] = sensordata[2];
	shoulder_angle_curve[499] = sensordata[3];
	pull_force_curve1[499] = sensordata[4];
	pull_force_curve2[499] = sensordata[5];
	pull_force_curve3[499] = sensordata[6];
	pull_force_curve4[499] = sensordata[7];
}

bool FatigueTest::IsErrorHappened() {
	if (m_pControlCard->elbow_position_in_degree < -20.0) {
		return true;
	}
	return false;
}

unsigned int __stdcall PressureThread(PVOID pParam) {
	FatigueTest *mTest = static_cast<FatigueTest *>(pParam);
	//ActiveMove *mMove = static_cast<ActiveMove *>(pParam);
	if (!mTest->IsInitialed()) {
		return 1;
	}

	//压力传感器
	ActiveMove::GetInstance().PressureSensorAcquisit();
}

unsigned int __stdcall SixdimThread(PVOID pParam) {
	FatigueTest *aTest = static_cast<FatigueTest *>(pParam);
	if (!aTest->IsInitialed()) {
		return 1;
	}

	//国产六维力
	ActiveMove::GetInstance().SixdimForceAcquisit();
}

unsigned int __stdcall TorqueThread(PVOID pParam) {
	FatigueTest *tTest = static_cast<FatigueTest *>(pParam);
	if (!tTest->IsInitialed()) {
		return 1;
	}

	//力矩传感器采集
	if (tTest->torque_collecting == true) {
		ActiveMove::GetInstance().TorqueAcquisit();
	}
}

unsigned int __stdcall FatigueTestThread(PVOID pParam) {
	FatigueTest *fTest = static_cast<FatigueTest *>(pParam);
	if (!fTest->IsInitialed()) {
		return 1;
	}

	while (fTest->is_fatigue) {
		ActiveMove::GetInstance().FatigueTestAcquisit();
	}

	fTest->ExportFatigueData();
}

void FatigueTest::ExitTorqueThread() {
	if (torque_thread != 0) {
		::WaitForSingleObject(torque_thread, INFINITE);
		torque_thread = 0;
	}
}

void FatigueTest::ExitSixdimForceThread() {
	if (sixdim_thread != 0) {
		::WaitForSingleObject(sixdim_thread, INFINITE);
		sixdim_thread = 0;
	}
}

void FatigueTest::ExitPressureThread() {
	if (pressure_thread != 0) {
		::WaitForSingleObject(pressure_thread, INFINITE);
		pressure_thread = 0;
	}
}

void FatigueTest::ExitFatigueThread() {
	if (fatiguetest_thread != 0) {
		::WaitForSingleObject(fatiguetest_thread, INFINITE);
		fatiguetest_thread = 0;
	}
}

void FatigueTest::ActiveTorqueToAllTorque(double torque[2], double alltorque[5]) {
	alltorque[0] = torque[0];
	alltorque[1] = torque[0] * 3 / 2;
	alltorque[2] = torque[1];
	alltorque[3] = torque[1] * 56 / 50;
	alltorque[4] = torque[1] * 74 / 50;
}

void FatigueTest::SetZero() {
	UpdataDataArray();
	PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);
}

void FatigueTest::ExportForceData() {
		ofstream dataFile1;
		dataFile1.open("farocedata.txt", ofstream::app);
		dataFile1 << "shoulderX" << "        " << "shoulderY"<<"        "<<"elbowX"<<"        "<<"elbowY" << endl;
		for (int i = 0; i < force_collection[0].size(); ++i) {
			dataFile1 << force_collection[0][i] << "        " << force_collection[1][i] << "        "<< force_collection[1][i]<<"        "<< force_collection[1][i]<<endl;
		}
		dataFile1.close();
}

void FatigueTest::ExportTorqueData() {
	ofstream dataFile2;
	dataFile2.open("torquedata.txt", ofstream::app);
	for (int i = 0; i < torque_collection[0].size(); ++i) {
		dataFile2 << torque_collection[0][i] << "        " << torque_collection[1][i] <<  endl;
	}
	dataFile2.close();
}

void FatigueTest::ExportMomentData() {
	ofstream dataFile3;
	dataFile3.open("momentdata.txt", ofstream::app);
	for (int i = 0; i < moment_collection[0].size(); ++i) {
		dataFile3 << moment_collection[0][i] << "        " << moment_collection[1][i] << endl;
	}
	dataFile3.close();
}

void FatigueTest::ExportSixDimensionData() {
	ofstream dataFile4;
	dataFile4.open("sixdimensiondata.txt", ofstream::app);
	for (int i = 0; i < sixdimension_collection.size(); ++i) {
		dataFile4 << sixdimension_collection[i] << endl;
	}
	dataFile4.close();
}

void FatigueTest::ExportFatigueData() {
	ofstream dataFile1;
	dataFile1.open("pulldata.txt", ofstream::app);
	for (int i = 0; i < fatigue_collection[0].size(); ++i) {
		dataFile1<< fatigue_collection[0][i] << "        " << fatigue_collection[1][i]
			<< "        "<< fatigue_collection[2][i] << "        " << fatigue_collection[3][i] << "        " << fatigue_collection[4][i] 
			<< "        " << fatigue_collection[5][i] << endl;
	}
	dataFile1.close();
}