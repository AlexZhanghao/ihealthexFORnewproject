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
