#pragma once
#include"fatigue_test.h"

class ActiveMove {
public:
	static ActiveMove& GetInstance();
	~ActiveMove();

	//ѹ���������ɼ�
	void PressureSensorAcquisit();
	//������ά��
	void SixdimForceAcquisit();
	//���ش������ɼ�
	void TorqueAcquisit();
	//ƣ�Ͳ����߳�,����߳�Ҫ��ϱ����˶�һ��ʹ��
	void FatigueTestAcquisit();
	void ActMove();

public:
	double two_arm_offset[8];
	double output_moment[8]{ 0.0 };

private:
	ActiveMove();

	//��ת�����ֵ�����˲�-���װ�����˹��ͨ�˲���
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	//������ѹ�����������ݽ����˲�
	void Trans2FilterForPressure(double TransData[4], double FiltedData[4]);
	//�������������ݴ����������άʸ��������ʸ��ֻ�����������������ã��������4�����ݡ�����Ҫ��֪���������İ�װλ��
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]);
	void MomentCalculation(double ForceVector[4], double vel[2]);

private:
	FatigueTest *m_fatigue;
	DataAcquisition *m_dataacquisition;
	ControlCard *m_controlcard;

	static double Ud_Arm;
	static double Ud_Shoul;
	static double six_dimforce[6];
};