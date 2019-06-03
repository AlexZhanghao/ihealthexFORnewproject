#pragma once
#include "NIDAQmx.h"

class DataAcquisition {
public:
	DataAcquisition() = default;
	void AcquisiteTorqueData();
	void AcquisitePullSensorData();
	void AcquisiteSixDemensionData(double output_buf[6]);

public:
	double torque_data[2] {0};
	double pull_sensor_data[4] {0};
private:
	const char *torque_channel = "dev3/ai4:5";
	const char *pull_sensor_channel = "Dev3/ai0:3";
	const char *six_dimension_force_channel = "Dev1/ai0:5";
};