#ifndef MARKER_H
#define MARKER_H

#include "EKF.hpp"
#include "DATA.hpp"
#include "SENSOR_MIN.hpp"
#include "CORE.hpp"
#include "CONSTANTS.hpp"

#include <armadillo>
using namespace arma;

class SENSOR;

// Struct defining each bag marker
class MARKER
{
public:
	vec position; // [m] Position vector of the marker
	vec position_true; // [m] True position vector of the marker (only known in simulation)
	int mid; // Marker ID
	bool plate; // Flag if marker contains a magnetic steel plate
	bool sim; // Software run mode flag
	bool updated = false; // Flag if marker's position was estimated. Used to pass on to orientation algorithm

	MARKER();
	MARKER(vec p, int m, int pl);
	~MARKER();
	void estimate_pos(vector<vector<DATA>>, vector<SENSOR_MIN>, CORE, CONSTANTS);
	void plot_ekf();
	void plot_e_y();

private:
	std::vector<EKF> ekf; // EKF results

	void init_ekf();
	EKF ekf_update(EKF, vector<SENSOR_MIN>, vector<DATA>, CORE, CONSTANTS);
	tuple<double, double> get_yhat(SENSOR_MIN, CORE);
	mat get_H_tilde(SENSOR_MIN, CORE core);
	
};

#endif /* MARKER_H */