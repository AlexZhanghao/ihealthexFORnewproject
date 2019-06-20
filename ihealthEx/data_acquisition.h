#pragma once
#include "NIDAQmx.h"

class DataAcquisition {
public:
	 DataAcquisition();
	 ~DataAcquisition();
	void AcquisiteTorqueData();
	void AcquisitePullSensorData();
	void AcquisiteSixDemensionData(double output_buf[6]);
	//这里尝试下把肩肘的数据采集放在一起，感觉这样性能可以提升
	void AcquisiteTensionData(double tension_output[8]);

	bool StartTask();
	bool StopTask();

public:
	double torque_data[2] {0};
	double pull_sensor_data[4] {0};




private:
	TaskHandle m_task_handle;
	const char *torque_channel = "dev3/ai4:5";
	const char *pull_sensor_channel = "Dev3/ai0:3";
	const char *six_dimension_force_channel = "Dev1/ai0:5";
	const char *kPressureForceChannel = "Dev2/ai1:8";
};