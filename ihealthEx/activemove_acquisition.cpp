#include"activemove_acquisition.h"
#include "Matrix.h"
#include<iostream>

using namespace std;

double ActiveMove::Ud_Arm = 0;
double ActiveMove::Ud_Shoul = 0;
double ActiveMove::six_dimforce[6]{ 0 };

ActiveMove &ActiveMove::GetInstance() {
	static ActiveMove instance;
	return instance;
}

ActiveMove::ActiveMove() {
	m_fatigue = new FatigueTest;
	m_dataacquisition = new DataAcquisition;
}

ActiveMove::~ActiveMove() {
	if (m_fatigue != NULL) {
		delete m_fatigue;
	}

	if (m_dataacquisition != NULL) {
		delete m_dataacquisition;
	}
}

void ActiveMove::PressureSensorAcquisit() {
	if (!m_fatigue->IsInitialed()) {
		return;
	}

	double pressure_data[8] = { 0 };
	double pull_data[4]{ 0 };
	double distData[8] = { 0 };
	double filtedData[8] = { 0 };
	double shoulder_suboffset[4] = { 0 };
	double elbow_suboffset[4] = { 0 };
	double shoulder_smooth[4] = { 0 };
	double elbow_smooth[4] = { 0 };
	double force_vector[4] = { 0 };
	double vel[2]{ 0 };

	//将肩肘部合在一起
	double two_arm_sum[8]{ 0.0 };
	double two_arm_buf[8]{ 0.0 };
	double two_arm_suboffset[8]{ 0.0 };

	for (int i = 0; i < 10; ++i) {
		m_dataacquisition->AcquisiteTensionData(two_arm_buf);
		for (int j = 0; j < 8; ++j) {
			two_arm_sum[j] += two_arm_buf[j];
		}
	}

	for (int i = 0; i < 8; ++i) {
		two_arm_offset[i] = two_arm_sum[i] / 10;
	}

	m_dataacquisition->StopTask();
	m_dataacquisition->StartTask();

	while (m_fatigue->IsMoved() == true) {
		m_dataacquisition->AcquisiteTensionData(pressure_data);


		for (int i = 0; i < 8; ++i) {
			two_arm_suboffset[i] = pressure_data[i] - two_arm_offset[i];
		}

		//因为滤波会用到之前的数据，所以在这里还是得把数据分开，同时把单位从电压转换成力
		for (int i = 0; i < 4; ++i) {
			shoulder_suboffset[i] = 20 * two_arm_suboffset[i];
		}
		for (int j = 0; j < 4; ++j) {
			elbow_suboffset[j] = 10 * two_arm_suboffset[j + 4];
		}

		//将传感器获取的数据滤波
		Trans2FilterForPressure(shoulder_suboffset, shoulder_smooth);
		Trans2FilterForPressure(elbow_suboffset, elbow_smooth);

		//将传感器数据转成力矢量
		SensorDataToForceVector(shoulder_suboffset, elbow_suboffset, force_vector);

		//将压力转化成关节力矩
		MomentCalculation(force_vector, vel);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("force_vector1:%lf    force_vector2:%lf    force_vector3:%lf	   force_vector4:%lf   \n", force_vector[0], force_vector[1], force_vector[2], force_vector[3]);

		Ud_Shoul = 3 * vel[0];
		Ud_Arm = 3.5 * vel[1];

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("Ud_Shoul:%lf  Ud_Arm:%lf\n", Ud_Shoul, Ud_Arm);

		//if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
		//	Ud_Arm = 0;
		//}
		//if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
		//	Ud_Shoul = 0;
		//}
		if (Ud_Arm > 3) {
			Ud_Arm = 3;
		}
		else if (Ud_Arm < -3) {
			Ud_Arm = -3;
		}
		if (Ud_Shoul > 3) {
			Ud_Shoul = 3;
		}
		else if (Ud_Shoul < -3) {
			Ud_Shoul = -3;
		}

		ActMove();

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout <<"m_shoulder_moment:\n"<< m_shoulder_moment << "\n" << "m_elbow_moment:\n"<< m_elbow_moment << endl;
		//cout << "shoulder_suboffset1:" << shoulder_suboffset[0] << "   " << "shoulder_suboffset2:" << shoulder_suboffset[1] << "   " << "shoulder_suboffset3:" << shoulder_suboffset[2] << "   " << "shoulder_suboffset4:" << shoulder_suboffset[3] << endl;
		//cout << "elbow_suboffset1:" << elbow_suboffset[0] << "   " << "elbow_suboffset2:" << elbow_suboffset[1] << "   " << "elbow_suboffset3:" << elbow_suboffset[2] << "   " << "elbow_suboffset4:" << elbow_suboffset[3] << endl;

		////采集力矩传感器数据
		//torque_collection[0].push_back(shoulder_torque);
		//torque_collection[1].push_back(elbow_torque);
		////采集六维力数据
		//sixdimension_collection.push_back(sixdim_elbow_moment);
		////采集压力传感器计算出的数据
		//moment_collection[0].push_back(m_shoulder_moment);
		//moment_collection[1].push_back(m_elbow_moment);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;

		Sleep(100);
	}
}

void ActiveMove::SixdimForceAcquisit() {
	if (!m_fatigue->IsInitialed()) {
		return;
	}
	double readings[6];
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };
	double sub_bias[6] = { 0 };

	// 求六维力传感器的偏置
	double sum[6]{ 0.0 };
	double buf[6]{ 0.0 };
	double six_dimension_offset_[6]{ 0 };
	for (int i = 0; i < 10; ++i) {
		m_dataacquisition->AcquisiteSixDemensionData(buf);
		for (int j = 0; j < 6; ++j) {
			sum[j] += buf[j];
		}
	}
	for (int i = 0; i < 6; ++i) {
		six_dimension_offset_[i] = sum[i] / 10;
	}

	while (true) {
		if (!m_fatigue->IsSixdimForceMoved()) {
			break;
		}

		m_dataacquisition->AcquisiteSixDemensionData(readings);

		// 求减去偏置之后的六维力，这里对z轴的力和力矩做了一个反向
		for (int i = 0; i < 6; ++i) {
			sub_bias[i] = readings[i] - six_dimension_offset_[i];
		}
		sub_bias[2] = -sub_bias[2];
		sub_bias[5] = -sub_bias[5];

		for (int i = 0; i < 6; ++i) {
			six_dimforce[i] = sub_bias[i];
		}

		Sleep(100);
	}
}

void ActiveMove::TorqueAcquisit() {
	if (!m_fatigue->IsInitialed()) {
		return;
	}

	double elbow_offset = 0;
	double shoulder_offset = 0;
	double shouleder_torque_sum_data = 0;
	double elbow_torque_sum_data = 0;
	double torquedata[2]{ 0 };

	//这里采到到的值会出现都是0的情况，所以加个检查
	while (elbow_torque_sum_data == 0) {
		m_dataacquisition->AcquisiteTorqueOffset();
		for (int j = 0; j < 5; ++j) {
			elbow_torque_sum_data += m_dataacquisition->torque_data[j + 5];
			shouleder_torque_sum_data += m_dataacquisition->torque_data[15 + j];
		}
	}

	elbow_offset = elbow_torque_sum_data / 5;
	shoulder_offset = shouleder_torque_sum_data / 5;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("elbow_offset:%lf  shoulder_offset:%lf\n", elbow_offset, shoulder_offset);

	m_dataacquisition->StopTask();
	m_dataacquisition->StartTask();

	while (m_fatigue->IsTested() == true) {
		m_dataacquisition->AcquisiteTorqueData(torquedata);
		m_fatigue->shoulder_torque = 2 * (torquedata[1] - shoulder_offset);
		m_fatigue->elbow_torque = -2 * (torquedata[0] - elbow_offset);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("elbow_offset:%lf    shoulder_offset:%lf  \n  shoulder_torque:%lf    elbow_torque:%lf  \n", elbow_offset, shoulder_offset, 2 * m_pDataAcquisition->torque_data[1], 2 * m_pDataAcquisition->torque_data[0]);

	}
}

void ActiveMove::FatigueTestAcquisit() {
	double torquedata[2]{ 0 };
	double angle[2]{ 0 };

	m_dataacquisition->AcquisiteTorqueData(torquedata);
	m_dataacquisition->AcquisitePullSensorData();
	m_controlcard->GetEncoderData(angle);

	m_fatigue->fatigue_collection[0].push_back(angle[0]);
	m_fatigue->fatigue_collection[1].push_back(angle[1]);
	m_fatigue->fatigue_collection[2].push_back(m_dataacquisition->pull_sensor_data[0]);
	m_fatigue->fatigue_collection[3].push_back(m_dataacquisition->pull_sensor_data[1]);
	m_fatigue->fatigue_collection[4].push_back(m_dataacquisition->pull_sensor_data[2]);
	m_fatigue->fatigue_collection[5].push_back(m_dataacquisition->pull_sensor_data[3]);
}

void ActiveMove::ActMove() {
	m_controlcard->VelocityMove(SHOULDER_AXIS_ID, Ud_Shoul);
	m_controlcard->VelocityMove(ELBOW_AXIS_ID, Ud_Arm);
}

void ActiveMove::Trans2Filter(double TransData[6], double FiltedData[6]) {
	double Wc = 5;
	double Ts = 0.05;
	static int i = 0;
	static double Last_Buffer[6] = { 0 };
	static double Last2_Buffer[6] = { 0 };
	static double Force_Buffer[6] = { 0 };
	static double Last_FT[6] = { 0 };
	static double Last2_FT[6] = { 0 };
	for (int m = 0; m < 6; m++) {
		if (i == 0) {
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else if (i == 1) {
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else {
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
				+ (2 * Wc*Wc)*Last_Buffer[m]
				+ (Wc*Wc)*Last2_Buffer[m]
				- (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
				- (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
}

void ActiveMove::Trans2FilterForPressure(double TransData[4], double FiltedData[4]) {
	double Wc = 5;
	double Ts = 0.1;
	static int i = 0;
	static double Last_Buffer[4] = { 0 };
	static double Last2_Buffer[4] = { 0 };
	static double Force_Buffer[4] = { 0 };
	static double Last_FT[4] = { 0 };
	static double Last2_FT[4] = { 0 };
	for (int m = 0; m < 4; m++)
	{
		if (i == 0)
		{
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else if (i == 1)
		{
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else
		{
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
				+ (2 * Wc*Wc)*Last_Buffer[m]
				+ (Wc*Wc)*Last2_Buffer[m]
				- (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
				- (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
}

void ActiveMove::SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]) {
	double shoulderdataY = shouldersensordata[0] - shouldersensordata[1];
	double shoulderdataZ = shouldersensordata[2] - shouldersensordata[3];
	double elbowdataY = elbowsensordata[0] - elbowsensordata[1];
	//这里因为8在Y+,7在Y-,所以用8-7表示正向
	double elbowdataZ = elbowsensordata[3] - elbowsensordata[2];

	//合成的力矢量
	//Vector2d shoulderforce;
	//Vector2d elbowforce;
	//shoulderforce << shoulderdataY, shoulderdataZ;
	//elbowforce << elbowdataY, elbowdataZ;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout <<"shoulderforce:\n"<< shoulderforce << "\n" << "elbowforce:\n"<<elbowforce << endl;

	ForceVector[0] = shoulderdataY;
	ForceVector[1] = shoulderdataZ;
	ForceVector[2] = elbowdataY;
	ForceVector[3] = elbowdataZ;
}

void ActiveMove::MomentCalculation(double ForceVector[4], double vel[2]) {
	MatrixXd m_vel(2, 1);
	MatrixXd pos(2, 1);

	VectorXd shoulder_force_moment_vector(6);
	VectorXd elbow_force_moment_vector(6);
	VectorXd six_dimensional_force_simulation(6);
	VectorXd v_moment(5);

	double angle[2];
	double moment[5];

	m_controlcard->GetEncoderData(angle);

	pos(0, 0) = angle[0];
	pos(1, 0) = angle[1];

	shoulder_force_moment_vector(0) = 0;
	shoulder_force_moment_vector(1) = ForceVector[0];
	shoulder_force_moment_vector(2) = ForceVector[1];
	shoulder_force_moment_vector(3) = 0;
	shoulder_force_moment_vector(4) = 0;
	shoulder_force_moment_vector(5) = 0;
	elbow_force_moment_vector(0) = 0;
	elbow_force_moment_vector(1) = ForceVector[2];
	elbow_force_moment_vector(2) = ForceVector[3];
	elbow_force_moment_vector(3) = 0;
	elbow_force_moment_vector(4) = 0;
	elbow_force_moment_vector(5) = 0;

	MomentBalance(shoulder_force_moment_vector, elbow_force_moment_vector, angle, moment);

	for (int i = 0; i < 5; ++i) {
		m_fatigue->moment_to_interface[i] = moment[i];
	}

	for (int i = 0; i < 5; ++i) {
		v_moment(i) = moment[i];
	}

	m_fatigue->m_shoulder_moment = moment[0];
	m_fatigue->m_elbow_moment = moment[2];

	AdmittanceControl(v_moment, m_vel);

	vel[0] = m_vel(0);
	vel[1] = m_vel(1);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("moment1:%lf      moment2:%lf      moment3:%lf      moment4:%lf      moment5:%lf  \n", moment[0], moment[1], moment[2], moment[3], moment[4]);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;
	//cout << "shoulderX:" << ForceVector[0] << "		" << "shoudlerY:" << ForceVector[1] << endl;
	//cout << "elbowX:" << ForceVector[2] << "		" << "elbowY:" << ForceVector[3] << endl;
}