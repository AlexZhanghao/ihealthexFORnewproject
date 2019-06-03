#include "fatigue_test.h"

#include <process.h>
#include <string>
#include <iostream>

#include "Matrix.h"

#define BOYDET_TIME 0.1

double Force_Fc = 0.3;
double Force_a = 0.3;
double Force_b = 1;
	
unsigned int __stdcall TestThread(PVOID pParam);
unsigned int __stdcall AcquisitionThread(PVOID pParam);

void FatigueTest::Initial(HWND hWnd) {
	m_hWnd = hWnd;
	m_pControlCard = new ControlCard();
	m_pControlCard->Initial();
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
	//主动运动线程
	test_thread = (HANDLE)_beginthreadex(NULL, 0, TestThread, this, 0, NULL);
	//传感器开启线程
	acquisition_thread= (HANDLE)_beginthreadex(NULL, 0, AcquisitionThread, this, 0, NULL);
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
		UpdataDataArray(readings);
		PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
}

void FatigueTest::StopMove() {
	in_test_move = false;
	//m_pControlCard->SetMotor(MOTOR_OFF);
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
		elbow_angle_error[i] = elbow_angle_error[i + 1];
		shoulder_angle_error[i] = shoulder_angle_error[i + 1];
		elbow_angle_curve[i] = elbow_angle_curve[i + 1];
		shoulder_angle_curve[i] = shoulder_angle_curve[i + 1];
		pull_force_curve1[i] = pull_force_curve1[i + 1];
		pull_force_curve2[i] = pull_force_curve2[i + 1];
		pull_force_curve3[i] = pull_force_curve3[i + 1];
		pull_force_curve4[i] = pull_force_curve4[i + 1];
		elbow_torque_curve[i] = elbow_torque_curve[i + 1];
		shoulder_torque_curve[i] = shoulder_torque_curve[i + 1];
	}
	elbow_angle_error[499] = m_pControlCard->elbow_error_in_degree;
	shoulder_angle_error[499] = m_pControlCard->shoulder_error_in_degree;
	elbow_angle_curve[499] = m_pControlCard->elbow_position_in_degree;
	shoulder_angle_curve[499] = m_pControlCard->shoulder_position_in_degree;
	pull_force_curve1[499] = m_pDataAcquisition->pull_sensor_data[0];
	pull_force_curve2[499] = m_pDataAcquisition->pull_sensor_data[1];
	pull_force_curve3[499] = m_pDataAcquisition->pull_sensor_data[2];
	pull_force_curve4[499] = m_pDataAcquisition->pull_sensor_data[3];
	elbow_torque_curve[499] = m_pDataAcquisition->torque_data[0];
	shoulder_torque_curve[499] = m_pDataAcquisition->torque_data[1];
}

void FatigueTest::UpdataDataArray(double buf[6]) {
	for (int i = 0; i < 499; i++) {
		elbow_angle_error[i] = elbow_angle_error[i + 1];
		shoulder_angle_error[i] = shoulder_angle_error[i + 1];
		elbow_angle_curve[i] = elbow_angle_curve[i + 1];
		shoulder_angle_curve[i] = shoulder_angle_curve[i + 1];
		pull_force_curve1[i] = pull_force_curve1[i + 1];
		pull_force_curve2[i] = pull_force_curve2[i + 1];
		/*pull_force_curve3[i] = pull_force_curve3[i + 1];
		pull_force_curve4[i] = pull_force_curve4[i + 1];
		elbow_torque_curve[i] = elbow_torque_curve[i + 1];
		shoulder_torque_curve[i] = shoulder_torque_curve[i + 1];*/
	}
	elbow_angle_error[499] = buf[0];
	shoulder_angle_error[499] = buf[1];
	elbow_angle_curve[499] = buf[2];
	shoulder_angle_curve[499] = buf[3];
	pull_force_curve1[499] = buf[4];
	pull_force_curve2[499] = buf[5];
	/*pull_force_curve3[499] = 
	pull_force_curve4[499] = m_pDataAcquisition->pull_sensor_data[3];
	elbow_torque_curve[499] = m_pDataAcquisition->torque_data[0];
	shoulder_torque_curve[499] = m_pDataAcquisition->torque_data[1];*/
}

void FatigueTest::AbsoluteMove() {
	if (!IsInitialed()) {
		return;
	}
	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);
	m_pControlCard->MotorAbsoluteMove(SHOULDER_AXIS_ID, 40, 2);
	m_pControlCard->MotorAbsoluteMove(ELBOW_AXIS_ID, 40, 2);
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
	mTest->StartMove();
}

unsigned int __stdcall AcquisitionThread(PVOID pParam) {
	FatigueTest *Bydetect = (FatigueTest*)pParam;
	UINT oldTickCount, newTickCount;
	oldTickCount = GetTickCount();
	while (TRUE)
	{
		if (Bydetect->m_stop)
			break;
		//延时 BOYDET_TIME s
		while (TRUE)
		{
			newTickCount = GetTickCount();
			if (newTickCount - oldTickCount >= BOYDET_TIME * 1000)
			{
				oldTickCount = newTickCount;
				break;
			}
			else
			{
				SwitchToThread();
				::Sleep(5);
			}
		}
		if (Bydetect->m_stop)
			break;
	}

	Bydetect->AcquisiteData();
}

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
	A.block(0, 3, 3, 1) = ForcePositionHat * rotate_matrix;
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
	VectorXd moment(6);
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
	TauExport(Pos,Six_Sensor_Convert, moment);
	m_shoulder_vel = Vel(0, 0);
	m_elbow_vel = Vel(1, 0);

	//printf("肩部速度: %lf\n", Ud_Shoul);
	//printf("肘部速度: %lf\n", Ud_Arm);

	char message_tracing[1024];
	if (m_elbow_vel > 5) {
		m_elbow_vel = 5;
	} else if (m_elbow_vel < -5) {
		m_elbow_vel = -5;
	}
	if (m_shoulder_vel > 5) {
		m_shoulder_vel = 5;
	} else if (m_shoulder_vel < -5) {
		m_shoulder_vel = -5;
	}
}