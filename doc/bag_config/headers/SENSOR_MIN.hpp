#ifndef SENSOR_MIN_H
#define SENSOR_MIN_H

#include "CORE.hpp"

#include <armadillo>
using namespace arma;

// Defining the SENSOR_MIN class
// Minimal Sensor objects contain only the information necessary to perform marker estimation
// Populated after calibration has been completed
class SENSOR_MIN
{
public:
	// Data:
	vec position; // [m] Position vector of the sensor
	vec point; // [m] Point that the sensor is pointing at
	mat frame; // Matrix defining the frame of the sensor
	mat Q_C; // DCM to convert from the sensor frame to the Core Frame
	
	const double x_max = 315; // Max x-measurement
	const double y_max = 207; // Max y-measurement
	const double gamma = 30; // [deg] Horizontal FOV half-angle
	const double theta = 20; // [deg] Vertical FOV half-angle
	
	mat R; // Measurement noise matrix
	double dT; // [s] Sampling rate
	int sid; // Sensor ID
	bool sim; // Flag defining simulation

	// Methods:
	SENSOR_MIN();
	SENSOR_MIN(vec pos, vec pnt);
	~SENSOR_MIN();
	mat get_sensor_frame(vec, vec, CORE);
	void set_params();

};

#endif