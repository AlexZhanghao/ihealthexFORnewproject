#pragma once
#include"Matrix.h"

class sensorprocess {
public:
	double DataFusion(double momentaverage, double torqueaverage,double momentvariance,double torquevariance);
	//�������������ݴ����������άʸ��������ʸ��ֻ�����������������ã��������4�����ݡ�����Ҫ��֪���������İ�װλ��
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]);

};