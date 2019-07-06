#pragma once
#include"Matrix.h"

class sensorprocess {
public:
	double DataFusion(double momentaverage, double torqueaverage,double momentvariance,double torquevariance);
	//将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]);

};