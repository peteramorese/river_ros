#include "BAG.hpp"

#include <armadillo>
#include <cmath>
using namespace arma;


BAG::BAG()
{
	string param_str;
	string warn_str;
	vector<double> pos_vec;

	// ros::VP_string remappings;
	// remappings.push_back(std::make_pair("", ""));
	// ros::init(remappings, "bag");

	for(int i = 0; i < 6; i++)
	{
		param_str = "/bag_config_node/bag_def/marker_";
		
		MARKER marker;
		marker.mid = i;
		if(i == 1 || i == 2)
		{
			marker.plate = 1;
		}

		param_str = param_str + to_string(marker.mid);
		if(CheckParam(param_str, 2) && CheckParamSize(param_str, 3))
		{
			ros::param::get(param_str, pos_vec);
			
			marker.position = {pos_vec[0], pos_vec[1], pos_vec[2]};
		}

		markers.push_back(marker);
	}

	pos_vec = {0, 0, 0};
	center = pos_vec;
};

BAG::BAG(int nm)
{
	for(int i = 0; i < nm; i++)
	{
		MARKER marker;
		marker.mid = i;
		markers.push_back(marker);
	}
}

BAG::~BAG(){};

mat BAG::euler_angle_to_dcm(double r, double p, double y)
{
	// Calculates the DCM corresponding to the 3-1-3 Euler Angle rotation
	//
	// Inputs:
	//		double r - [deg] Roll Euler Angle
	//		double p - [deg] Pitch Euler Angle
	// 		double y - [deg] Yaw Euler Angle
	//
	// Outputs:
	// 		mat Q - [3x3] Direction Cosine Matrix corresponding to the 3-1-3 rotation

	r = r * M_PI / 180.0;
	p = p * M_PI / 180.0;
	y = y * M_PI / 180.0;

	mat Q(3, 3);

	Q = {{cos(p), sin(p)*sin(y), cos(y)*sin(p)},
		{sin(r)*sin(p), cos(r)*cos(y) - cos(p)*sin(r)*sin(y), -cos(r)*sin(y) - cos(p)*cos(y)*sin(r)},
		{-cos(r)*sin(p), cos(y)*sin(r) + cos(r)*cos(p)*sin(y), cos(r)*cos(p)*cos(y) - sin(r)*sin(y)}};

	return Q;
};

void BAG::move_bag(vec x, double r, double p, double y)
{
	// Function to move the center of the bag and rotate the bag using 3-1-3 Euler Angle definitions
	// 
	// Inputs:
	//		vec x - [3x1] [m] New position vector of the center of the bag
	//		double r - [deg] Roll Euler Angle
	//		double p - [deg] Pitch Euler Angle
	// 		double y - [deg] Yaw Euler Angle

	center = x; // Reassign the center position of the bag

	mat Q = euler_angle_to_dcm(r, p, y); // Get the DCM for the rotation

	for(int i = 0; i < markers.size(); i++) // For each marker
	{
		markers[i].position = Q * markers[i].position;
		markers[i].position = markers[i].position + center;
	}
};


void BAG::estimate_ori()
{
	std::vector<std::pair<double[3], int>> bag_pair = get_bag_pair(true);

	oriDetermine od(bag_pair);

	std::vector<std::pair<double[3], int>> coords_3D = get_coords_3D();

	od.setData(coords_3D);
	od.setIterations(10000);
	matrix<double> matout(3, 3);
	matout = od.returnQ();
	matout.print();

	mat Q;
	vec curr;

	Q = {{matout(0, 0), matout(0, 1), matout(0, 2)},
		{matout(1, 0), matout(1, 1), matout(1, 2)}, 
		{matout(2, 0), matout(2, 1), matout(2, 2)}};

	DCM = Q;

	std::vector<std::pair<double[3], int>> bag_def = get_bag_pair(true);

	// Compute the rotated (and centered) positions of each marker
	for(int i = 0; i < bag_def.size(); i++)
	{
		curr = {bag_def[i].first[0], bag_def[i].first[1], bag_def[i].first[2]};
		curr = Q * curr;

		bag_def[i].first[0] = curr[0];
		bag_def[i].first[1] = curr[1];
		bag_def[i].first[2] = curr[2];
	}

	// Find the first estimated marker index
	int Index;

	for(int i = 0; i < markers.size(); i++)
	{
		if(markers[i].updated == true)
		{
			Index = i;
			break;
		}
	}

	vec move;
	move = {markers[Index].position[0] - bag_def[Index].first[0], 
		markers[Index].position[1] - bag_def[Index].first[1],
		markers[Index].position[2] - bag_def[Index].first[2]};

	center = move;

	string param_str = "/bag_config_node/verbose";
	bool verbose = true;
	if(CheckParam(param_str, 1, verbose))
	{
		ros::param::get(param_str, verbose);
	}

	if(verbose)
	{
		cout << "Center: " << move[0] << "\t\t" << move[1] << "\t\t" << move[2] << endl;
	}

	for(int i = 0; i < bag_def.size(); i++)
	{
		if(markers[i].updated == false)
		{
			curr = {bag_def[i].first[0], bag_def[i].first[1], bag_def[i].first[2]};

			markers[i].position = curr + move;
		}

		if(verbose)
		{
			cout << "M" << i << "  " << markers[i].position[0] << "  " << markers[i].position[1] << "  " << markers[i].position[2] << endl;
		}
	}

	quat = dcm_to_quat(Q.t());

	vec Core2UR = {-0.421, 0, 1.037};

	center = center + Core2UR;
}


std::array<double, 4> BAG::dcm_to_quat(mat Q)
{
	cout << "Entering Quat" << endl;

	std::array<double, 4> quat;

	double q1, q2, q3, q4;
	double q1_sq, q2_sq, q3_sq, q4_sq;
	q1_sq = 0.25*(1 + Q(0, 0) - Q(1, 1) - Q(2, 2));
	q2_sq = 0.25*(1 - Q(0, 0) + Q(1, 1) - Q(2, 2));
	q3_sq = 0.25*(1 - Q(0, 0) - Q(1, 1) + Q(2, 2));
	q4_sq = 0.25*(1 + Q(0, 0) + Q(1, 1) + Q(2, 2));

	if(q1_sq >= q2_sq && q1_sq >= q3_sq && q1_sq >= q4_sq)
	{
		q1 = sqrt(q1_sq);
		q2 = 1/(4*q1) * (Q(0,1) + Q(1, 0));
		q3 = 1/(4*q1) * (Q(2, 0) + Q(0, 2));
		q4 = 1/(4*q1) * (Q(1, 2) - Q(2, 1));
	}
	else if(q2_sq >= q1_sq && q2_sq >= q3_sq && q2_sq >= q4_sq)
	{
		q2 = sqrt(q2_sq);
		q1 = 1/(4*q2) * (Q(0,1) + Q(1, 0));
		q3 = 1/(4*q2) * (Q(1, 2) + Q(2, 1));
		q4 = 1/(4*q2) * (Q(2, 0) - Q(0, 2));
	}
	else if(q3_sq >= q1_sq && q3_sq >= q2_sq && q3_sq >= q4_sq)
	{
		q3 = sqrt(q3_sq);
		q1 = 1/(4*q3) * (Q(2, 0) + Q(0, 2));
		q2 = 1/(4*q3) * (Q(1, 2) + Q(2, 1));
		q4 = 1/(4*q3) * (Q(0, 1) - Q(1, 0));
	}
	else if(q4_sq >= q1_sq && q4_sq >= q2_sq && q4_sq >= q3_sq)
	{
		q4 = sqrt(q4_sq);
		q1 = 1/(4*q4) * (Q(1, 2) - Q(2, 1));
		q2 = 1/(4*q4) * (Q(2, 0) - Q(0, 2));
		q3 = 1/(4*q4) * (Q(0, 1) - Q(1, 0));
	}
	else
	{
		cout << "There was an error calculating the quaternion from the DCM:" << endl;
		cout << "Q = \t[" << Q(0, 0) << "\t" << Q(0, 1) << "\t" << Q(0, 2) << "]" << endl;
		cout << "\t[" << Q(1, 0) << "\t" << Q(1, 1) << "\t" << Q(1, 2) << "]" <<  endl;
		cout << "\t[" << Q(2, 0) << "\t" << Q(2, 1) << "\t" << Q(2, 2) << "]" << endl;
	}


	quat = {q1, q2, q3, q4};
	cout << "Exiting Quat" << endl;
	return quat;
}


std::vector<std::pair<double[3], int>> BAG::get_bag_pair(bool def)
{
	std::vector<std::pair<double[3], int>> bag_def;
	std::vector<std::pair<double[3], int>> bag_pair;

	// CHANGE THE BAG DEFINITION FOR THE REAL THING YOU DUMB BITCH
	bag_def.resize(6);

	string param_str = "/bag_config_node/bag_def/num_markers";
	double num_mark = 6;
	if(CheckParam(param_str, 1, num_mark))
	{
		ros::param::get(param_str, num_mark);
	}

	for(int i = 0; i < num_mark; i++)
	{
		param_str = "/bag_config_node/bag_def/marker_";
		param_str = param_str + to_string(i);
		vector<double> pos_vec;

		if(CheckParam(param_str, 2) && CheckParamSize(param_str, 3))
		{
			ros::param::get(param_str, pos_vec);
			bag_def[i].first[0] = pos_vec[0];
			bag_def[i].first[1] = pos_vec[1];
			bag_def[i].first[2] = pos_vec[2];
			bag_def[i].second = i;
		}
	}

	vector<int> mark;

	for(int i = 0; i < markers.size(); i++)
	{
		if(markers[i].updated == true || def)
		{
			mark.push_back(i);
		}
	}

	for(int i = 0; i < mark.size(); i++)
	{
		bag_pair.push_back(bag_def[mark[i]]);
	}

	return bag_pair;
}


std::vector<std::pair<double[3], int>> BAG::get_coords_3D()
{
	std::vector<std::pair<double[3], int>> coords_3D;
	int Nmark = 0;
	vector<int> mark;

	for(int i = 0; i < markers.size(); i++)
	{
		if(markers[i].updated == true)
		{
			Nmark++;
			mark.push_back(i);
		}
	}

	coords_3D.resize(Nmark);

	for(int i = 0; i < Nmark; i++)
	{
		coords_3D[i].first[0] = markers[mark[i]].position[0];
		coords_3D[i].first[1] = markers[mark[i]].position[1];
		coords_3D[i].first[2] = markers[mark[i]].position[2];
		coords_3D[i].second = markers[mark[i]].mid;

//		cout << "i = " << mark[i] << "  " << coords_3D[i].second << endl;
	}

	return coords_3D;
}
