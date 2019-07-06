#include"sensordata_processing.h"
#include <iostream>
#include<fstream>
#include<vector>

double sensorprocess::DataFusion(double momentaverage, double torqueaverage, double momentvariance, double torquevariance) {
	double fusion = 0;
	momentvariance = momentvariance * momentvariance;
	torquevariance = torquevariance * torquevariance;
	fusion = (momentaverage*torquevariance + torqueaverage * momentvariance) / (momentvariance + torquevariance);
	return fusion;
}

void sensorprocess::SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]) {
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
