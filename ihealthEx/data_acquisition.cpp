#include "data_acquisition.h"

#include <Eigen/core>

using namespace Eigen;

void DataAcquisition::AcquisiteTorqueData() {
	TaskHandle  taskHandle = 0;
	int32       read = 0;
	int status = 0;

	status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, torque_channel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, torque_data, 2, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);
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
	status = DAQmxCreateTask("SixDemensionDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, six_dimension_force_channel, "PullDataChannel", DAQmx_Val_Diff, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, raw_data, 6, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);

	//º∆À„
	Matrix<double, 6, 6> m;
	m << 0.08729, -0.00137, 0.59422, 47.13000, 0.06298, -47.92455,
		0.77953, -54.66462, 0.13566, 27.25556, 0.51927, 27.55145,
		65.33104, 1.23854, 64.00936, -0.70825, 65.17850, -1.71349,
		-0.06931, -0.07835, -1.20443, 0.02817, 1.08440, 0.01389,
		1.34213, 0.01130, -0.59052, -0.04531, -0.61663, 0.05584,
		-0.01694, 1.06263, 0.00710, 1.01083, -0.00028, 0.94187;
	Matrix<double, 6, 1> dat;
	for (int i = 0; i < 6; ++i) {
		dat(i, 0) = raw_data[i];
	}

	VectorXd result(6);
	result = m * dat;

	//ºı»•∆´÷√
	VectorXd bias(6);
	bias << 12.0295, 18.6972, -82.3837, -1.92185, 1.24067, 0.6300;
	result = result - bias;

	for (int i = 0; i < 6; ++i) {
		output_buf[i] = result(i);
	}
}