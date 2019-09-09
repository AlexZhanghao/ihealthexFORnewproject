#include "data_acquisition.h"
#include<iostream>
#include <Eigen/core>
#include<Windows.h>

using namespace Eigen;
using namespace std;

const double DataAcquisition::kRawToReal = 2.0;
Eigen::Matrix<double, 6, 6> DataAcquisition::kTransformMatrix = MatrixXd::Zero(6, 6);

DataAcquisition::DataAcquisition() {
	int status;
	//status = DAQmxCreateTask("", &m_task_handle);
	//status = DAQmxCreateAIVoltageChan(m_task_handle, kPressureForceChannel, "",
	//								  DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(m_task_handle, NULL, 100, DAQmx_Val_Rising,
	//							   DAQmx_Val_ContSamps, 2);

	//status = DAQmxSetReadRelativeTo(m_task_handle, DAQmx_Val_MostRecentSamp);
	//status = DAQmxSetReadOffset(m_task_handle, 0);
	//status = DAQmxStartTask(m_task_handle);
	//status = DAQmxStopTask(m_task_handle);

	//六维力
	status = DAQmxCreateTask("", &s_task_handle);
	status = DAQmxCreateAIVoltageChan(s_task_handle, six_dimension_force_channel, "",
		DAQmx_Val_Diff, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(s_task_handle, NULL, 100, DAQmx_Val_Rising,
		DAQmx_Val_ContSamps, 2);

	status = DAQmxSetReadRelativeTo(s_task_handle, DAQmx_Val_MostRecentSamp);
	status = DAQmxSetReadOffset(s_task_handle, 0);
	status = DAQmxStartTask(s_task_handle);
	status = DAQmxStopTask(s_task_handle);

	//初始化变换矩阵
	kTransformMatrix << 0.09692, -0.10635, 0.52011, 47.49211, 0.15094, -47.35115,
		0.22184, -54.45397, -0.35906, 27.02136, 0.21803, 27.46034,
		64.42479, -0.39683, 63.49878, -0.09020, 65.20982, -1.84716,
		-0.00948, -0.00277, -1.15283, 0.00340, 1.14574, -0.03061,
		1.29493, -0.02196, -0.68483, -0.01380, -0.63174, 0.00974,
		-0.00283, 0.98940, 0.00518, 1.00811, -0.00211, 1.00804;
}

DataAcquisition::~DataAcquisition() {

}

void DataAcquisition::AcquisiteTorqueData(double torquedata[2]) {
	TaskHandle  taskHandle = 0;
	int32  read = 0;
	int status = 0;

	status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, torque_channel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, torquedata, 2, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("shoulder:%lf     elbow:%lf\n", torquedata[1], torquedata[0]);
}

void DataAcquisition::AcquisiteTorqueOffset() {
	TaskHandle  taskHandle = 0;
	int32  read = 0;
	int status = 0;

	status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, torque_channel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 10, 0.2, DAQmx_Val_GroupByChannel, torque_data, 20, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("shoulder:%lf     elbow:%lf\n", torque_data[15], torque_data[5]);
}

void DataAcquisition::AcquisitePullSensorData() {
	TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;

	status = DAQmxCreateTask("PullDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, pull_sensor_channel, "PullDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, pull_sensor_data, 4, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);
}

void DataAcquisition::AcquisiteSixDemensionData(double output_buf[6]) {
	TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	double raw_data[6];
	/*status = DAQmxCreateTask("SixDemensionDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, kSixDimensionForceChannel, "SixDimensionChannel", DAQmx_Val_Diff, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 100, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, raw_data, 6, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);*/
	status = DAQmxReadAnalogF64(s_task_handle, 1, 0.2, DAQmx_Val_GroupByScanNumber, raw_data, 6, &read, NULL);
	//cout << "status is : " << status << endl;
	//char buffer[256];
	//DAQmxGetErrorString(status, buffer, 256);
	//cout << "error code is : " << buffer << endl;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("%lf    %lf    %lf    %lf    %lf    %lf \n", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5]);

	Matrix<double, 6, 1> dat;
	for (int i = 0; i < 6; ++i) {
		dat(i, 0) = raw_data[i];
	}

	VectorXd result(6);
	result = kTransformMatrix * dat;


	for (int i = 0; i < 6; ++i) {
		output_buf[i] = result(i);
	}
}

void DataAcquisition::AcquisiteTensionData(double tension_output[8]) {
	int32 read = 0;
	int status = 0;
	double tension_data[8]{ 0 };
	//status = DAQmxCreateTask("PressureDataTask", &taskHandle);
	//status = DAQmxCreateAIVoltageChan(taskHandle, kPressureForceChannel, "PressureDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising,DAQmx_Val_ContSamps, 2);
	//status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(m_task_handle, 1, 0.2, DAQmx_Val_GroupByScanNumber, tension_data, 8, &read, NULL);
	//status = DAQmxStopTask(taskHandle);
	//status = DAQmxClearTask(taskHandle);

	for (int i = 0; i < 8; ++i) {
		tension_output[i] = tension_data[i];
	}
}

bool DataAcquisition::StartTask() {
	int status;
	status = DAQmxStartTask(m_task_handle);
	return status == 0;
}

bool DataAcquisition::StopTask() {
	int status;
	status = DAQmxStopTask(m_task_handle);
	return status == 0;
}

double DataAcquisition::ShoulderForwardPull() {
	return kRawToReal * pull_sensor_data[0];
}

double DataAcquisition::ShoulderBackwardPull() {
	return kRawToReal * pull_sensor_data[1];
}

double DataAcquisition::ElbowForwardPull() {
	return kRawToReal * pull_sensor_data[2];
}

double DataAcquisition::ElbowBackwardPull() {
	return kRawToReal * pull_sensor_data[3];
}
