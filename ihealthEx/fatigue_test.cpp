#include "fatigue_test.h"
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

const double shoulder_moment_variance = 4.3769 / 10000;
const double elbow_moment_variance = 1.2576 / 10000;
const double shoulder_torque_variance = 1.4634 / 100000;
const double elbow_torque_variance = 2.2896 / 100000;
const double shoulder_pos = -20;
const double elbow_pos = -20;
const double sixdim = 0.015;
double sixdim_shoulder_moment;
	
vector<double> force_data[4];
vector<double> torque_data[2];
vector<double> moment_data[2];

unsigned int __stdcall TestThread(PVOID pParam);
unsigned int __stdcall ATIThread(PVOID pParam);
//unsigned int __stdcall AcquisitionThread(PVOID pParam);

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

void FatigueTest::StartTest() {

	//ATI六维力传感器使用
	mFTWrapper.LoadCalFile();
	mFTWrapper.BiasCurrentLoad(true);
	mFTWrapper.setFUnit();
	mFTWrapper.setTUnit();

	is_testing = true;

	//主动运动线程
	test_thread = (HANDLE)_beginthreadex(NULL, 0, TestThread, this, 0, NULL);
	//ATI线程启动，这里要小心线程冲突
	ATI_thread = (HANDLE)_beginthreadex(NULL, 0, ATIThread, this, 0, NULL);
	////传感器开启线程
	//acquisition_thread= (HANDLE)_beginthreadex(NULL, 0, AcquisitionThread, this, 0, NULL);
}

void FatigueTest::StartAbsoulteMove() {
	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);

	is_moving = true;

	AbsoluteMove();
}

void FatigueTest::AbsoluteMove() {
	if (!IsInitialed()) {
		return;
	}
	I32 Axis[2] = { SHOULDER_AXIS_ID,ELBOW_AXIS_ID };

	//APS_ptp_v(Axis[0], option, shoulder_pos / VEL_TO_PALSE, -3 /  VEL_TO_PALSE, NULL);
	//APS_ptp_v(Axis[1], option, elbow_pos / VEL_TO_PALSE, -3 / VEL_TO_PALSE, NULL);
	APS_absolute_move(Axis[0], shoulder_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
	APS_absolute_move(Axis[1], elbow_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
}

void FatigueTest::timerAcquisit() {
	if (!IsInitialed()) {
		return;
	}

	double readings[7] = { 0 };
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };

	m_pDataAcquisition->AcquisiteTorqueData();
	elbow_offset = 2*m_pDataAcquisition->torque_data[0];
	shoulder_offset = 2*m_pDataAcquisition->torque_data[1];

	while (true) {
		mFTWrapper.GetForcesAndTorques(readings);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//std::cout << "f0: " << readings[0] << " f1: " << readings[1] <<
		//	" f2: " << readings[2] << " f3: " << readings[3] <<
		//	" f4: " << readings[4] << " f5: " << readings[5] << std::endl;

		//Raw2Trans(readings, distData);
		//Trans2Filter(distData, filtedData);
		//FiltedVolt2Vel(filtedData);

		SixDimensionForceRotation(readings);

		sixdim_shoulder_moment = readings[5];

		UpdataDataArray2(readings);
		PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
}

void FatigueTest::SixDimensionForceRotation(double sixdimensionforce[6]) {
	VectorXd sixdim(6);
	Vector3d sixdim_to_coordinate3 = Vector3d(d3 - shouler_installationsite_to_coordinate4, sixdim_shoulder, 0);

	for (int i = 0; i < 6; ++i) {
		sixdim(i) = sixdimensionforce[i];			
	}
	
	MatrixXd sixdim_transfer(6,6);
	Matrix3d sixdim_rotation;
	Matrix3d to_zero;
	Matrix3d Sixdim_To_Coordinate3;

	VectorToMatrix(sixdim_to_coordinate3, Sixdim_To_Coordinate3);
	to_zero.setZero();

	sixdim_rotation <<
		1, 0, 0,
		0, -1, 0,
		0, 0, -1;

	sixdim_transfer <<
		sixdim_rotation, to_zero,
		Sixdim_To_Coordinate3*sixdim_rotation, sixdim_rotation;

	sixdim = sixdim_transfer*sixdim;
	for (int i = 0; i < 6; ++i) {
		sixdimensionforce[i] = sixdim(i);
	}
}

void FatigueTest::StartMove() {
	if (!IsInitialed()) {
		return;
	}
	in_test_move = true;
	double readings[6];
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };

	while (true) {
		if (!in_test_move) {
			break;
		}

		m_pDataAcquisition->AcquisiteSixDemensionData(readings);

		for (int i = 0; i < 6; ++i) {
			readings[i] = -readings[i];
		}

		Raw2Trans(readings, distData);
		Trans2Filter(distData, filtedData);
		FiltedVolt2Vel(filtedData);



		//注意这里我们传入的顺序是fxfyfzMxMyMz,现在传的是裸值，以后如果你要传入
		//经过转换后的值的话，传入的顺序也要是fxfyfzMxMyMz(转换后的值力和力矩的位置是交换了的)
		//UpdataDataArray(moments);
		//PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
}

void FatigueTest::StopMove() {
	in_test_move = false;
	is_testing = false;
	m_pControlCard->SetMotor(MOTOR_OFF);
	//这里不放开离合的原因是为了防止中间位置松开离合导致手臂迅速下坠
	//m_pControlCard->SetClutch(CLUTCH_OFF);
}

void FatigueTest::PositionReset() {
	if (!IsInitialed()) {
		return;
	}
	m_pControlCard->PositionReset();
}

void FatigueTest::AcquisiteData() {
	m_pDataAcquisition->AcquisiteTorqueData();
	m_pDataAcquisition->AcquisitePullSensorData();
	m_pControlCard->GetEncoderData();
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
		//elbow_torque_curve[i] = elbow_torque_curve[i + 1];
		//shoulder_torque_curve[i] = shoulder_torque_curve[i + 1];
	}
	//elbow_angle_error[499] = m_pControlCard->elbow_error_in_degree;
	//shoulder_angle_error[499] = m_pControlCard->shoulder_error_in_degree;
	//elbow_angle_curve[499] = m_pControlCard->elbow_position_in_degree;
	//shoulder_angle_curve[499] = m_pControlCard->shoulder_position_in_degree;
	//pull_force_curve1[499] = m_pDataAcquisition->pull_sensor_data[0];
	//pull_force_curve2[499] = m_pDataAcquisition->pull_sensor_data[1];
	//pull_force_curve3[499] = m_pDataAcquisition->pull_sensor_data[2];
	//pull_force_curve4[499] = m_pDataAcquisition->pull_sensor_data[3];
	//elbow_torque_curve[499] = m_pDataAcquisition->torque_data[0];
	//shoulder_torque_curve[499] = m_pDataAcquisition->torque_data[1];
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

unsigned int __stdcall TestThread(PVOID pParam) {
	FatigueTest *mTest = static_cast<FatigueTest *>(pParam);
	if (!mTest->IsInitialed()) {
		return 1;
	}

	//国产六维力
	//mTest->StartMove();
	//压力传感器
	mTest->PressureSensorAcquisit();
}

unsigned int __stdcall ATIThread(PVOID pParam) {
	FatigueTest *aTest = static_cast<FatigueTest *>(pParam);
	if (!aTest->IsInitialed()) {
		return 1;
	}

	//ATI六维力
	aTest->timerAcquisit();
}

//unsigned int __stdcall AcquisitionThread(PVOID pParam) {
//	FatigueTest *Bydetect = (FatigueTest*)pParam;
//	UINT oldTickCount, newTickCount;
//	oldTickCount = GetTickCount();
//	while (TRUE)
//	{
//		if (Bydetect->m_stop)
//			break;
//		//延时 BOYDET_TIME s
//		while (TRUE)
//		{
//			newTickCount = GetTickCount();
//			if (newTickCount - oldTickCount >= BOYDET_TIME * 1000)
//			{
//				oldTickCount = newTickCount;
//				break;
//			}
//			else
//			{
//				SwitchToThread();
//				::Sleep(5);
//			}
//		}
//		if (Bydetect->m_stop)
//			break;
//	}
//
//	Bydetect->AcquisiteData();
//	Bydetect->shoulder_moment = Bydetect->m_pDataAcquisition->torque_data[0];
//	Bydetect->elbow_moment = Bydetect->m_pDataAcquisition->torque_data[1];
//}

void FatigueTest::Raw2Trans(double RAWData[6], double DistData[6]) {
	//这一段就是为了把力从六维力传感器上传到手柄上，这里的A就是总的一个转换矩阵。
	//具体的旋转矩阵我们要根据六维力的安装确定坐标系方向之后然后再确定。
	MatrixXd A(6, 6);
	A.setZero();
	VectorXd Value_Origi(6);
	VectorXd Value_Convers(6);
	Matrix3d rotate_matrix;
	//这里的旋转矩阵要根据六维力坐标系和手柄坐标系来具体得到
	rotate_matrix <<
		cos(M_PI_4), sin(M_PI_4), 0,
		-sin(M_PI_4), cos(M_PI_4), 0,
		0, 0, 1;
	//Vector3d ForcePosition(-0.075,0.035,0);
	//手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
	Vector3d ForcePosition(0.075, -0.035, 0);
	Matrix3d ForcePositionHat;
	//这里就是这个p，我们可以想象，fx不会产生x方向的力矩，fy产生的看z坐标，fz产生的y坐标。
	//这里做的就是把力矩弄过去。这个相对坐标都是六维力坐标在手柄坐标系下的位置。
	//比如fx在y方向上有一个力臂，就会产生一个z方向上的力矩。这个力矩的方向和相对位置无关。
	//所以这个地方我们不用改这个ForcePositionHat，只用改ForcePosition这个相对位置就可以了
	ForcePositionHat <<
		0, -ForcePosition[2], ForcePosition[1],
		ForcePosition[2], 0, -ForcePosition[0],
		-ForcePosition[1], ForcePosition[0], 0;
	A.block(0, 0, 3, 3) = rotate_matrix;
	A.block(0, 3, 3, 3) = ForcePositionHat * rotate_matrix;
	A.block(3, 3, 3, 3) = rotate_matrix;


	//之前是fxfyfzMxMyMz,现在变成MxMyMzfxfyfz
	for (int i = 0; i < 6; i++) {
		if (i<3) {
			Value_Origi(i) = RAWData[i + 3];
		} else {
			Value_Origi(i) = RAWData[i - 3];
		}
	}

	//这里计算后就是换算到手柄坐标系上之后的六维力的值，MxMyMzfxfyfz
	Value_Convers = A * Value_Origi;
	for (int m = 0; m<6; m++) {
		DistData[m] = Value_Convers(m);
	}
}

void FatigueTest::Trans2Filter(double TransData[6], double FiltedData[6]) {
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
		} else if (i == 1) {
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		} else {
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

void FatigueTest::FiltedVolt2Vel(double FiltedData[6]) {
	MatrixXd Vel(2, 1);
	MatrixXd Pos(2, 1);
	MatrixXd A(6, 6);
	VectorXd Six_Sensor_Convert(6);
	VectorXd moment(5);
	double angle[2];
	m_pControlCard->GetEncoderData(angle);
	Pos(0, 0) = angle[0];
	Pos(1, 0) = angle[1];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("elbow angle: %lf\n", angle[1]);
	//printf("shoulder angle: %lf\n", angle[0]);
	//printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1], FiltedData[2]);

	for (int i = 0; i < 6; i++) {
		Six_Sensor_Convert(i) = FiltedData[i];
	}
	damping_control(Six_Sensor_Convert, Pos, Vel, Force_Fc, Force_a, Force_b);

	//计算tau值并输出
	TauExport(Pos, Six_Sensor_Convert, moment);
	shoulder_tau = moment(0);
	elbow_tau = moment(2);

	//m_pDataAcquisition->AcquisiteTorqueData();
	//shoulder_moment = 2 * m_pDataAcquisition->torque_data[1] - shoulder_offset;
	//elbow_moment = -(2 * m_pDataAcquisition->torque_data[0] - elbow_offset);

	//shoulder_difference = shoulder_tau - shoulder_moment;
	//elbow_difference = elbow_tau - elbow_moment;

	//moments[0] = shoulder_moment;
	//moments[1] = elbow_moment;
	//moments[2] = shoulder_tau;
	//moments[3] = elbow_tau;
	//moments[4] = shoulder_difference;
	//moments[5] = elbow_difference;
	//
	//UpdataDataArray(moments);
    //PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

	//m_shoulder_vel = Vel(0, 0);
	//m_elbow_vel = Vel(1, 0);

	//printf("肩部速度: %lf\n", Ud_Shoul);
	//printf("肘部速度: %lf\n", Ud_Arm);
}

void FatigueTest::PressureSensorAcquisit() {
	if (!IsInitialed()) {
		return;
	}

	double pressure_data[8] = { 0 };
	double pull_data[4] { 0 };
	double distData[8] = { 0 };
	double filtedData[8] = { 0 };
	double shoulder_suboffset[4] = { 0 };
	double elbow_suboffset[4] = { 0 };
	double shoulder_smooth[4] = { 0 };
	double elbow_smooth[4] = { 0 };
	double force_vector[4] = { 0 };
	
	//将肩肘部合在一起
	double two_arm_sum[8] { 0.0 };
	double two_arm_buf[8] { 0.0 };
	double two_arm_suboffset[8]{ 0.0 };

	//换成出的和传感器测得的力矩,这里只有6个数据，但是测压力传感器数据时有八个数据，
	//所以这里多加两个和那个匹配，只是最后两个是恒为0的
	double output_moment[8]{ 0.0 };

	for (int i = 0; i < 10; ++i) {
		m_pDataAcquisition->AcquisiteTensionData(two_arm_buf);
		for (int j = 0; j < 8; ++j) {
			two_arm_sum[j] += two_arm_buf[j];
		}
	}

	for (int i = 0; i < 8; ++i) {
		two_arm_offset[i] = two_arm_sum[i] / 10;
	}

	m_pDataAcquisition->AcquisiteTorqueData();
	elbow_offset = 2 * m_pDataAcquisition->torque_data[0];
	shoulder_offset = 2 * m_pDataAcquisition->torque_data[1];

	m_pDataAcquisition->StopTask();
	m_pDataAcquisition->StartTask();


	while (is_testing==true){
		m_pDataAcquisition->AcquisiteTensionData(pressure_data);

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

		//因为肘部第二位置没有传感器，所以这里把它强制置零
		elbow_suboffset[1] = 0;

		//将传感器获取的数据滤波
		Trans2Filter2(shoulder_suboffset, shoulder_smooth);
		Trans2Filter2(elbow_suboffset, elbow_smooth);

		//将传感器数据转成力矢量
		SensorDataToForceVector(shoulder_smooth, elbow_smooth, force_vector);

		MomentCalculation(force_vector);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout <<"m_shoulder_moment:\n"<< m_shoulder_moment << "\n" << "m_elbow_moment:\n"<< m_elbow_moment << endl;
		//cout << "shoulder_suboffset1:" << shoulder_suboffset[0] << "   " << "shoulder_suboffset2:" << shoulder_suboffset[1] << "   " << "shoulder_suboffset3:" << shoulder_suboffset[2] << "   " << "shoulder_suboffset4:" << shoulder_suboffset[3] << endl;
		//cout << "elbow_suboffset1:" << elbow_suboffset[0] << "   " << "elbow_suboffset2:" << elbow_suboffset[1] << "   " << "elbow_suboffset3:" << elbow_suboffset[2] << "   " << "elbow_suboffset4:" << elbow_suboffset[3] << endl;

		m_pDataAcquisition->AcquisiteTorqueData();
		shoulder_moment = 2 * m_pDataAcquisition->torque_data[1] - shoulder_offset;
		elbow_moment = -(2 * m_pDataAcquisition->torque_data[0] - elbow_offset);

		//采集力矩传感器数据
		torque_data[0].push_back(shoulder_moment);
		torque_data[1].push_back(elbow_moment);

		//shoulder_difference = m_shoulder_moment - shoulder_moment;
		//elbow_difference = m_elbow_moment - elbow_moment;

		shoulder_fusion = m_psensorprocess.DataFusion(m_shoulder_moment, shoulder_moment, shoulder_moment_variance, shoulder_torque_variance);
		elbow_fusion = m_psensorprocess.DataFusion(m_elbow_moment, elbow_moment, elbow_moment_variance, elbow_torque_variance);

		output_moment[0] = m_shoulder_moment;
		output_moment[1] = m_elbow_moment;
		output_moment[2] = sixdim_shoulder_moment;
		output_moment[3] = elbow_fusion;
		output_moment[4] = shoulder_moment;
		output_moment[5] = elbow_moment;



		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;
		
		//UpdataDataArray2(output_moment);
		//PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
	ExportMomentData();
	ExportTorqueData();
	//ExportForceData();
}

void FatigueTest::Trans2Filter2(double TransData[4], double FiltedData[4]) {
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

void FatigueTest::MomentCalculation(double ForceVector[4]) {
	MatrixXd vel(2, 1);
	MatrixXd pos(2, 1);

	VectorXd shoulder_force_moment_vector(6);
	VectorXd elbow_force_moment_vector(6);
	VectorXd six_dimensional_force_simulation(6);

	double angle[2];
	double moment[5];

	m_pControlCard->GetEncoderData(angle);

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
		moment1[i] = moment[i];
	}

	moment1[6] = moment[0] / moment[2];
	moment1[7] = moment[1] / moment[2];

	if (moment1[6] > 5 || moment1[6] < -5) {
		moment1[6] = 0;
	}
	if (moment1[7] > 5 || moment1[7] < -5) {
		moment1[7] = 0;
	}

	//采集压力传感器数据
	moment_data[0].push_back(moment[0]);
	moment_data[1].push_back(moment[2]);

	m_shoulder_moment = moment[0];
	m_elbow_moment = moment[2];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;
}

void FatigueTest::SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]) {
	double shoulderdataX = shouldersensordata[0] - shouldersensordata[1];
	double shoulderdataY = shouldersensordata[2] - shouldersensordata[3];
	double elbowdataX = elbowsensordata[0] - elbowsensordata[1];
	double elbowdataY = elbowsensordata[2] - elbowsensordata[3];

	//合成的力矢量
	Vector2d shoulderforce;
	Vector2d elbowforce;
	shoulderforce << shoulderdataX, shoulderdataY;
	elbowforce << elbowdataX, elbowdataY;

	//force_data[0].push_back(shoulderdataX);
	//force_data[1].push_back(shoulderdataY);
	//force_data[2].push_back(elbowdataX);
	//force_data[3].push_back(elbowdataY);

	//将力分别旋转到坐标系3、5上面
	//Matrix2d shoulderrotationmatrix;
	//Matrix2d elbowrotationmatrix;
	//shoulderrotationmatrix << cos(29.49* M_PI / 180), cos(60.51* M_PI / 180),
	//	cos(119.49* M_PI / 180), cos(29.49* M_PI / 180);
	//elbowrotationmatrix << cos(59.87* M_PI / 180), cos(30.13* M_PI / 180),
	//	cos(30.13* M_PI / 180), cos(120.13* M_PI / 180);

	//shoulderforce = shoulderrotationmatrix * shoulderforce;
	//elbowforce = elbowrotationmatrix * elbowforce;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout <<"shoulderforce:\n"<< shoulderforce << "\n" << "elbowforce:\n"<<elbowforce << endl;

	ForceVector[0] = shoulderforce(0);
	ForceVector[1] = shoulderforce(1);
	ForceVector[2] = elbowforce(0);
	ForceVector[3] = elbowforce(1);
}

void FatigueTest::SetZero() {
	UpdataDataArray();
	PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);
}

void FatigueTest::ExportForceData() {
		ofstream dataFile1;
		dataFile1.open("farocedata.txt", ofstream::app);
		dataFile1 << "shoulderX" << "        " << "shoulderY"<<"        "<<"elbowX"<<"        "<<"elbowY" << endl;
		for (int i = 0; i < force_data[0].size(); ++i) {
			dataFile1 << force_data[0][i] << "        " << force_data[1][i] << "        "<< force_data[1][i]<<"        "<< force_data[1][i]<<endl;
		}
		dataFile1.close();
}

void FatigueTest::ExportTorqueData() {
	ofstream dataFile2;
	dataFile2.open("torquedata.txt", ofstream::app);
	for (int i = 0; i < torque_data[0].size(); ++i) {
		dataFile2 << torque_data[0][i] << "        " << torque_data[1][i] <<  endl;
	}
	dataFile2.close();
}

void FatigueTest::ExportMomentData() {
	ofstream dataFile3;
	dataFile3.open("momentdata.txt", ofstream::app);
	for (int i = 0; i < moment_data[0].size(); ++i) {
		dataFile3 << moment_data[0][i] << "        " << moment_data[1][i] << endl;
	}
	dataFile3.close();
}