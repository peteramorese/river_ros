#ifndef BAG_H
#define BAG_H

#include "MARKER.hpp"
#include "ERRORS.hpp"
#include "oriDetermine.h"
#include "matrix.h"
#include "ros/ros.h"

#include <string>
#include <sstream>
#include <armadillo>
using namespace arma;


// Class defining a cargo bag
class BAG
{
public:
	vec center;
	double length = 0.41; // [m] Cargo bag length
	double width = 0.23; // [m] Cargo bag width
	double height = 0.24; // [m] Cargo bag height
	std::vector<MARKER> markers;
	mat DCM;
	std::array<double, 4> quat;
	bool bag_found = false;

	BAG();
	BAG(int nm);
	~BAG();
	void move_bag(vec x, double roll, double pitch, double yaw);
	mat euler_angle_to_dcm(double roll, double pitch, double yaw);
	void estimate_ori();

private:
	std::vector<std::pair<double[3], int>> get_bag_pair(bool def);
	std::vector<std::pair<double[3], int>> get_coords_3D();
	std::array<double, 4> dcm_to_quat(mat);
};

#endif /* BAG_H */