#include "SENSOR_MIN.hpp"

#include <vector>
#include <armadillo>
using namespace arma;


SENSOR_MIN::SENSOR_MIN()
{
	// Initializer

	position = zeros<vec>(3);
	point = zeros<vec>(3);
	frame = zeros<mat>(3, 3);
	Q_C = zeros<mat>(3, 3);

	set_params(); // Set sensor parameters
}


SENSOR_MIN::SENSOR_MIN(vec pos, vec pnt)
{
	// Initializer

	position = pos;
	point = pnt;
	CORE c;
	frame = get_sensor_frame(pos, pnt, c);
	Q_C = c.frame * frame;

	set_params(); // Set sensor parameters
}


SENSOR_MIN::~SENSOR_MIN(){};


mat SENSOR_MIN::get_sensor_frame(vec pos, vec pnt, CORE c)
{
	// Compute the frame matrix defining the pointing of the sensor
	// 
	// Inputs:
	// 		pos - [vec] [3x1] [m] Position vector to initialize with
	// 		pnt - [vec] [3x1] [m] Pointing vector to initialize with
	// 		c - [CORE] Object defining the Core Frame
	// 
	// Output:
	// 		S - [mat] [3x3] Matrix defining the sensor frame
	// 

	// Z-axis (sensor frame) points directly at the point pnt
	vec z = pnt - pos;
	z = normalise(z);

	// X-axis (sensor frame) is normal to both the sensor Z-axis and the Core Z-axis
	// Corresponds to the sensor being level to the ground
	vec x = cross(z, c.frame.col(2));
	x = normalise(x);

	// Y-axis (sensor frame) completes the right handed coordinate system
	vec y = cross(z, x);

	mat S(3, 3);

	S.col(0) = x;
	S.col(1) = y;
	S.col(2) = z;

	return S;
}


void SENSOR_MIN::set_params()
{
	// Set the sensor parameters

	R = 8*eye(2, 2); // Measurement noise covariance matrix
	dT = 1.0/60.0; // [s] Sampling rate
}
