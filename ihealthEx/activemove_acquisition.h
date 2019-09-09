#pragma once
#include"fatigue_test.h"

class ActiveMove {
public:
	static ActiveMove& GetInstance();
	~ActiveMove();

	//压力传感器采集
	void PressureSensorAcquisit();
	//国产六维力
	void SixdimForceAcquisit();
	//力矩传感器采集
	void TorqueAcquisit();
	//疲劳测试线程,这个线程要配合被动运动一起使用
	void FatigueTestAcquisit();
	void ActMove();

public:
	double two_arm_offset[8];
	double output_moment[8]{ 0.0 };

private:
	ActiveMove();

	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	//用来对压力传感器数据进行滤波
	void Trans2FilterForPressure(double TransData[4], double FiltedData[4]);
	//将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
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