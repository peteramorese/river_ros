#include "SENSOR.hpp"

#include "matplotlibcpp.h"

#include <vector>
#include <cmath>
#include <armadillo>
#include <iostream>
#include <string>
#include <sstream>
using namespace arma;
namespace plt = matplotlibcpp;


SENSOR::SENSOR()
{
	// Initializer for the SENSOR class

	position = zeros<vec>(3);
	position_true = zeros<vec>(3);
	point = zeros<vec>(3);
	frame = zeros<mat>(3, 3);
	Q_C = zeros<mat>(3, 3);

	set_params(); // Set sensor parameters
}


SENSOR::SENSOR(vec pos, vec pnt)
{
	// Initializer for the SENSOR class

	position = pos;
	position_true = zeros<vec>(3);
	point = pnt;
	CORE c;
	frame = get_sensor_frame(pos, pnt, c);
	Q_C = c.frame * frame;

	set_params(); // Set the sensor parameters
}


SENSOR::~SENSOR(){};


void SENSOR::init_default_sensor(int id, CORE c)
{
	// Function to initalize the default positions and point of the sensor system
	// Positions and pointing must be defined here for operation
	// Positions will be calibrated more precisely later
	//
	// Inputs:
	//		int id - Sensor ID to initiate
	//		CORE c - Struct defining the Core Frame

	sid = id;
	string param_str;
	string param_substr = "/bag_config_node/sensor_";
	param_substr = param_substr + to_string(sid) + "/";
	string warn_str;

	vector<double> pos_vec;
	vector<double> pnt_vec;

	// Sensor Position
	param_str = param_substr + "position";
	CheckParam(param_str, 2);
	CheckParamSize(param_str, 3);
	ros::param::get(param_str, pos_vec);
	
	position = {pos_vec[0], pos_vec[1], pos_vec[2]};

	// Sensor Pointing
	param_str = param_substr + "point";
	CheckParam(param_str, 2);
	CheckParamSize(param_str, 3);
	ros::param::get(param_str, pnt_vec);
	
	point = {pnt_vec[0], pnt_vec[1], pnt_vec[2]};

	frame = get_sensor_frame(position, point, c); // Compute the sensor's pointing frame

	// Initialize the first EKF step
	EKF e;
	e.x_hat = position;
	e.P = 0.25*eye(3, 3);
	if(sim == true)
	{
		e.e_x = position - position_true;
	}

	ekf.push_back(e);
}


mat SENSOR::get_sensor_frame(vec pos, vec pnt, CORE c)
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


void SENSOR::set_params()
{
	// Set the sensor parameters

	R = 8*eye(2, 2); // Measurement noise covariance matrix
	dT = 1.0/60.0; // [s] Sampling rate
}


void SENSOR::calibrate_sensor(std::vector<std::vector<DATA>> Y, BAG bag, CORE core, CONSTANTS cnst)
{
	// Function to calibrate the sensor's position based on measurements of markers at known positions
	// Uses an Extended Kalman Filter to estimate
	// 
	// Inputs:
	// 		vector<double> t - [s] Time vector
	// 		vector<vector<DATA>> Y - Packets of measurements from the sensor being calibrated of multiple markers for time t
	// 		BAG bag - Object defining the known positions of the markers being measured
	// 		CORE core - Object defining the Core Frame
	// 		CONSTANTS cnst - Object defining system-wide constants
	// 


	EKF e;
	std::vector<DATA> y_k;

	for(int k = 0; k < Y.size(); k++) // For each time k
	{
		y_k = Y[k]; // Measurement of all markers at time k

		e = ekf_update(ekf[ekf.size()-1], bag, y_k, core, cnst); // Get next EKF update

		ekf.push_back(e); // Add EKF update to the sensor
	}
}


EKF SENSOR::ekf_update(EKF e, BAG bag, std::vector<DATA> y_k, CORE core, CONSTANTS cnst)
{
	// Function to perform an Extended Kalman Filter update given the previous EKF results and new measurements
	// 
	// Inputs:
	// 		EKF e - The previous estimation results of the EKF
	// 		vector<DATA> y_k - New measurements from the sensor at time k of each marker
	// 		CORE core - Object defining the Core Frame
	// 
	// Outputs:
	// 		EKF e_upd - Updated EKF step
	// 

	EKF e_kp1;
	int num_meas = 0; // Number of measurements (corresponds to one for each marker)
	std::vector<int> mid_valid;

	for(int i = 0; i < y_k.size(); i++)
	{
		if(isnan(y_k[i].x) == 0)
		{
			num_meas++; // Measurement is valid
			mid_valid.push_back(y_k[i].mid); // Measurement is of Marker MID
		}
	}

	if(num_meas > 0) // Measurement update
	{
		std::tuple<double, double> yhat_tup;
		vec yhat(cnst.p*num_meas);
		vec y(cnst.p*num_meas);
		mat H_tilde;
		mat R_tmp;
		mat R_blk;

		for(int i = 0; i < num_meas; i++)
		{
			// cout << "Marker " << bag.markers[mid_valid[i]].position << endl;
			yhat_tup = get_yhat(bag.markers[mid_valid[i]], core); // Get predicted measurement

			yhat[2*i] = std::get<0>(yhat_tup); // Store yhat in a stacked vector
			yhat[2*i+1] = std::get<1>(yhat_tup);

			y[2*i] = y_k[i].x; // Store real measurements in a stacked vector
			y[2*i+1] = y_k[i].y;

			mat H = get_H_tilde(bag.markers[mid_valid[i]]); 
			H_tilde = std::move(arma::join_cols(H_tilde, H));

			R_tmp = std::move(arma::join_rows(zeros(2, R_blk.n_cols), R));
			R_blk = std::move(arma::join_rows(R_blk, zeros(R_blk.n_rows, 2)));
			R_blk = std::move(arma::join_cols(R_blk, R_tmp));
		}

		e_kp1.e_y = y - yhat; // Measurement Innovation

		// Kalman gain matrix
		mat K_tilde = e.P * H_tilde.t() * (H_tilde * e.P * H_tilde.t() + R_blk).i();
		
		e_kp1.x_hat = e.x_hat + K_tilde * e_kp1.e_y; // New state estimate

		mat I = eye(cnst.n, cnst.n); // Identity

		e_kp1.P = (I - K_tilde * H_tilde)*e.P; // New covariance matrix


	}
	else // No measurement update
	{
		e_kp1.x_hat = e.x_hat;
		e_kp1.P = e.P;
	}

	if(sim == true) // If the software is running in simulation mode
	{
		e_kp1.e_x = e_kp1.x_hat - position_true;
	}

	position = e_kp1.x_hat; // Update sensor position
	frame = get_sensor_frame(position, point, core); // Update pointing frame
	Q_C = core.frame * frame; // Update DCM

	return e_kp1;
}


std::tuple<double, double> SENSOR::get_yhat(MARKER m, CORE c)
{
	// The nonlinear measurement function for yhat
	//
	// Inputs:
	//		MARKER m - Marker being measured
	//		CORE c - Object defining the Core Frame
	//		bool sim - Flag for whether data is simulated or real expected
	//
	// Outputs:
	//		tuple yhat - Pixy output [x, y]


	mat C = c.frame; // Core Frame matrix

	double gamma_rad = gamma * M_PI / 180.0; // [rad] Horizontal FOV half-angle
	double theta_rad = theta * M_PI / 180.0; // [rad] Vertical FOV half-angle

	// Constant matrices / vector
	mat T = {{tan(gamma_rad), 0, 0},
		{0, tan(theta_rad), 0},
		{0, 0, 1}};

	mat DELTA = {{2.0/x_max, 0},
		{0, 2.0/y_max},
		{0, 0}};

	vec ivec = {1.0, 1.0, -1.0};

	vec pos; // Position vector to calculate with

	pos = position;

	vec V_S = frame.t() * C.t() * (m.position - pos);
	double a = V_S[2];

	mat H = (1.0/a) * (DELTA.t() * DELTA).i() * DELTA.t() * T.i() * frame.t() * C.t();
	mat G = -(1.0/a) * (DELTA.t() * DELTA).i() * DELTA.t() * T.i() * frame.t() * C.t() * pos + (DELTA.t() * DELTA).i() * DELTA.t() * ivec;

	vec yhat_vec = H * m.position + G; // Predicted measurement yhat

	std::tuple<double, double> yhat;

	std::get<0>(yhat) = yhat_vec[0]; // Assign measurements to a tuple
	std::get<1>(yhat) = yhat_vec[1];

	return yhat;
}


void SENSOR::plot_ekf()
{
	// Plot the results of the Extended Kalman Filter (EKF) used to calibrate the sensor position

	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;

	std::vector<double> sigx;
	std::vector<double> sigy;
	std::vector<double> sigz;
	std::vector<double> nsigx;
	std::vector<double> nsigy;
	std::vector<double> nsigz;

	for(int i = 1; i < ekf.size()-1; i++)
	{
		if(sim == true) // If software run mode is simulation
		{
			x.push_back(ekf[i].e_x(0)); // Plot estimation error
			y.push_back(ekf[i].e_x(1));
			z.push_back(ekf[i].e_x(2));

			sigx.push_back(2.0*sqrt(ekf[i].P(0, 0))); // 2 Sigma bounds
			sigy.push_back(2.0*sqrt(ekf[i].P(1, 1)));
			sigz.push_back(2.0*sqrt(ekf[i].P(2, 2)));
			nsigx.push_back(-2.0*sqrt(ekf[i].P(0, 0)));
			nsigy.push_back(-2.0*sqrt(ekf[i].P(1, 1)));
			nsigz.push_back(-2.0*sqrt(ekf[i].P(2, 2)));
		}
		else // If software run mode is not simulation
		{
			x.push_back(ekf[i].x_hat(0)); // Plot state estimate
			y.push_back(ekf[i].x_hat(1));
			z.push_back(ekf[i].x_hat(2));

			sigx.push_back(ekf[i].x_hat(0) + 2.0*sqrt(ekf[i].P(0, 0))); // 2 Sigma bounds + estimate
			sigy.push_back(ekf[i].x_hat(1) + 2.0*sqrt(ekf[i].P(1, 1)));
			sigz.push_back(ekf[i].x_hat(2) + 2.0*sqrt(ekf[i].P(2, 2)));
			nsigx.push_back(ekf[i].x_hat(0) - 2.0*sqrt(ekf[i].P(0, 0)));
			nsigy.push_back(ekf[i].x_hat(1) - 2.0*sqrt(ekf[i].P(1, 1)));
			nsigz.push_back(ekf[i].x_hat(2) - 2.0*sqrt(ekf[i].P(2, 2)));
		}


	}

	// Figure formatting and plotting

	std::stringstream ttl;

	if(sim == true)
	{
		ttl << "Estimation Error vs No. of Measurement Updates - Sensor " << sid << endl;
	}
	else
	{
		ttl << "Estimation vs No. of Measurement Updates - Sensor " << sid << endl;
	}

	// plt::figure();
	plt::figure_size(1000, 1000);
	plt::suptitle(ttl.str(), {{"fontsize", "20"}});
	plt::subplot(3, 1, 1);
		if(sim == true)
		{
			plt::ylabel("Est. Error - X [m]", {{"fontsize", "16"}});
		}
		else
		{
			plt::ylabel("Estimate - X [m]", {{"fontsize", "16"}});
		}
		plt::grid(true);
		plt::named_plot("Est. Error", x, "b-");
		plt::named_plot("2$\\sigma$", sigx, "r-");
		plt::plot(nsigx, "r-");
		if(sim == true)
		{
			plt::ylim(5*nsigx.back(), 5*sigx.back());
		}
		else
		{
			// plt::ylim(x.back() + 5*(nsigx.back() - x.back()), x.back() + 5*(sigx.back() - x.back()));
		}
		plt::legend();
	plt::subplot(3, 1, 2);
		if(sim == true)
		{
			plt::ylabel("Est. Error - Y [m]", {{"fontsize", "16"}});
		}
		else
		{
			plt::ylabel("Estimate - Y [m]", {{"fontsize", "16"}});
		}
		plt::grid(true);
		plt::plot(y, "b-");
		plt::plot(sigy, "r-");
		plt::plot(nsigy, "r-");
		if(sim == true)
		{
			plt::ylim(5*nsigy.back(), 5*sigy.back());
		}
		else
		{
			// plt::ylim(y.back() + 5*(nsigy.back() - y.back()), y.back() + 5*(sigy.back() - y.back()));
		}
	plt::subplot(3, 1, 3);
		if(sim == true)
		{
			plt::ylabel("Est. Error - Z [m]", {{"fontsize", "16"}});
		}
		else
		{
			plt::ylabel("Estimate - Z [m]", {{"fontsize", "16"}});
		}
		plt::xlabel("No. of Measurement Updates [k]", {{"fontsize", "16"}});
		plt::grid(true);
		plt::plot(z, "b-");
		plt::plot(sigz, "r-");
		plt::plot(nsigz, "r-");
		if(sim == true)
		{
			plt::ylim(5*nsigz.back(), 5*sigz.back());
		}
		else
		{
			// plt::ylim(z.back() + 5*(nsigz.back() - z.back()), z.back() + 5*(sigz.back() - z.back()));
		}
	plt::show();
}


void SENSOR::plot_e_y()
{
	// Plot the measurement innovation results from the calibration Extended Kalman Filter

	std::vector<double> x;
	std::vector<double> y;
	// std::vector<double> z;

	std::vector<double> sigx;
	std::vector<double> sigy;
	// std::vector<double> sigz;

	for(int i = 1; i < ekf.size(); i++)
	{
		x.push_back(ekf[i].e_y(0));
		y.push_back(ekf[i].e_y(1));
		// z.push_back(ekf[i].e_y(2));
	}

	std::stringstream ttl;
	ttl << "Measurement Innovation vs Measurement Updates - Sensor " << sid << endl;

	plt::figure();
	plt::suptitle(ttl.str(), {{"fontsize", "20"}});
	plt::subplot(2, 1, 1);
	plt::plot(x, "b-");
	plt::ylabel("X Measurement Innovation", {{"fontsize", "16"}});

	plt::subplot(2, 1, 2);
	plt::plot(y, "b-");
	plt::ylabel("Y Measurement Innovation", {{"fontsize", "16"}});
	plt::xlabel("No. of Measurement Updates [k]", {{"fontsize", "16"}});
	plt::show();

	// plt::figure(3);
	// plt::plot(z, "b-");
	// plt::show();
}


mat SENSOR::get_H_tilde(MARKER m)
{
	// Function to find the H_tilde matrix for calibration of the sensor positioning system
	// Jacobian was computed symbolically in Mathematica and hand-typed here
	// Results were compared between this and the Mathematica output
	// 
	// Inputs:
	// 		MARKER m - Object containing the position of the marker being measured
	// 
	// Outputs:
	// 		mat H - H_tilde matrix used in calibration, equal to dh/ds
	// 

	// cout << "x = \n" << position << endl;
	// cout << "m = \n" << m.position << endl;

	double gamma_rad = gamma * M_PI / 180.0; // [rad] Horizontal FOV half-angle
	double theta_rad = theta * M_PI / 180.0; // [rad] Vertical FOV half-angle

	mat T = {{tan(gamma_rad), 0, 0},
			{0, tan(theta_rad), 0},
			{0, 0, 1}};

	mat DELTA = {{2.0/x_max, 0},
			{0, 2.0/y_max},
			{0, 0}};

	double s1 = position[0];
	double s2 = position[1];
	double s3 = position[2];

	double p1 = point[0];
	double p2 = point[1];
	double p3 = point[2];

	double x1 = m.position[0];
	double x2 = m.position[1];
	double x3 = m.position[2];

	// The H_tilde_partial matrix was computed symbolically in Mathematica, and hand-typed here
	mat H_tilde_partial(3, 3, arma::fill::zeros);

	H_tilde_partial(0, 0) = -(((-((s2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
             sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)
               )) - (s1*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))\
           + ((p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))\
           + ((-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)))
         *(-((p1 - s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          (pow(p1 - s1,2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s1 + x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p1 - s1)*(p2 - s2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          ((p1 - s1)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-((s1*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
             ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
         sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))) - 
      (s2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
      (s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + (s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) - (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
      (((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
           ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*x1)/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + ((-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
         x2)/sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
          2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)))/(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(0, 1) = -(((-((s2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
             sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)
               )) - (s1*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))\
           + ((p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))\
           + ((-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)))
         *(-((p2 - s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          ((p1 - s1)*(p2 - s2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          (pow(p2 - s2,2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s2 + x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p2 - s2)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-((s2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
             (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
         sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))) - 
      (s1*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
           ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
      (s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) - (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
      (s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + (((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
           ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
         x1)/sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
          2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + ((-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*x2)/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
            (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))
              )*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)))/(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(0, 2) = ((s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + (s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) - (s2*(-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      (s1*((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
           (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*
         (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + (((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
           (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*x1)/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) - 
      ((2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
            ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
              (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*
         (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
       (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
          1.5)) + ((-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
           (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*x2)/
       sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
         pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)))/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
   ((-((s2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
           sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)))
          - (s1*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))/
         sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
        ((p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x1)/
         sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
        ((-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*x2)/
         sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)))*
      (-((p3 - s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
        ((p1 - s1)*(p3 - s3)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
        ((p2 - s2)*(p3 - s3)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
        (-s3 + x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        (pow(p3 - s3,2)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
    pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2);

     H_tilde_partial(1, 0) = -(((-((-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                        s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                      pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                       2))*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
               pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
               (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
               pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
               (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
               pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3) - 
          s1*((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
          s2*((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
          ((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x1 + 
          ((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x2 + 
          (-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
             pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x3)*
        (-((p1 - s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          (pow(p1 - s1,2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s1 + x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p1 - s1)*(p2 - s2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          ((p1 - s1)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
           (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
      (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      ((-2*pow(p1,2)*(p1 - s1))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p1 - s1))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*(p1 - s1)*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p1 - s1)*pow(s1,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*(p1 - s1)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p1 - s1)*pow(s2,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (pow(p1,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3 + 
      (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      s2*((2*p2*p3*(p1 - s1))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p1 - s1)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p2*p3*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p1 - s1)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p1 - s1)*s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
      s1*((2*p1*p3*(p1 - s1))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p1 - s1)*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p1*p3*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         p3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p1 - s1)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p1 - s1)*s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         s3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
      ((2*p1*p3*(p1 - s1))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p1 - s1)*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p1*p3*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         p3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p1 - s1)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p1 - s1)*s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         s3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x1 + 
      ((2*p2*p3*(p1 - s1))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p1 - s1)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p2*p3*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p1 - s1)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p1 - s1)*s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x2 + 
      ((-2*pow(p1,2)*(p1 - s1))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p1 - s1))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*(p1 - s1)*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p1 - s1)*pow(s1,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*(p1 - s1)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p1 - s1)*pow(s2,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (pow(p1,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-((p1*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 ((p1 - s1)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p1 - s1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p1 - s1)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x3)/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(1, 1) = -(((-((-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                        s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                      pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                       2))*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
               pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
               (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
               pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
               (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
               pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3) - 
          s1*((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
          s2*((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
          ((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x1 + 
          ((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x2 + 
          (-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
             pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x3)*
        (-((p2 - s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          ((p1 - s1)*(p2 - s2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          (pow(p2 - s2,2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s2 + x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p2 - s2)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
              2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
           (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
      (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      ((-2*pow(p1,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*s1*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s1,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*(p2 - s2)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p2 - s2)*pow(s2,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (pow(p1,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3 + 
      (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      s1*((2*p1*p3*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s1*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p1*p3*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s1*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
      s2*((2*p2*p3*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p2 - s2)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p2*p3*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         p3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p2 - s2)*s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         s3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
      ((2*p1*p3*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s1*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p1*p3*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s1*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x1 + 
      ((2*p2*p3*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*(p2 - s2)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (p2*p3*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         p3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p2 - s2)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*(p2 - s2)*s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         s3/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x2 + 
      ((-2*pow(p1,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*s1*(p2 - s2))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s1,2)*(p2 - s2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*(p2 - s2)*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*(p2 - s2)*pow(s2,2))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (pow(p1,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-((p1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))*
               (-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
              2*((p2*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 ((p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x3)/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(1, 2) = (pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
            2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
      pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
      pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
      pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
           pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
         (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
      ((pow(p1,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*pow(p1,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*s1*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s1,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*s2*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s2,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)))*s3 - 
      s1*(-(p1/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
         s1/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*p3*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p1*p3*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s1*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s1*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2))) - 
      s2*(-(p2/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
         s2/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*p3*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p2*p3*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s2*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s2*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2))) + 
      (-(p1/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                 2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                 2))*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
         s1/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*p3*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p1*p3*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s1*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p1*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s1*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)))*x1 + 
      (-(p2/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                 2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
                 2))*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
         s2/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*p3*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (p3*s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (2*p2*p3*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*p3*s2*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (p2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))*s3)/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*p2*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (2*s2*(p3 - s3)*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)))*x2 + 
      ((pow(p1,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(p2,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p1*s1*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s1,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (p2*s2*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
         (pow(s2,2)*(2*(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                 s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               (-((p1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
                 (s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) + 
              2*(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
               ((p2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
                 (s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))))/
          (2.*pow(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2),
             1.5)*(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
         (2*pow(p1,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(p2,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p1*s1*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s1,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) + 
         (4*p2*s2*(p3 - s3))/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
              pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
            pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)) - 
         (2*pow(s2,2)*(p3 - s3))/
          (sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),
               2) + pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
             *pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),2)))*x3)/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
   ((-((-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                      s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                    pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))
                   *(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
             pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
             (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
             pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*s3) - 
        s1*((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
        s2*((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) + 
        ((p1*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p3*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (s1*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x1 + 
        ((p2*p3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p3*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           (p2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (s2*s3)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x2 + 
        (-(pow(p1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                    s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                  pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
                (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))) - 
           pow(p2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (2*p1*s1)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           pow(s1,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
           (2*p2*s2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
           pow(s2,2)/(sqrt(pow(-(p1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
                  s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2) + 
                pow(p2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2))*
              (pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))))*x3)*
      (-((p3 - s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
        ((p1 - s1)*(p3 - s3)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
        ((p2 - s2)*(p3 - s3)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
        (-s3 + x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        (pow(p3 - s3,2)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
    pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2);

     H_tilde_partial(2, 0) = -(((-(((p1 - s1)*s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
          ((p2 - s2)*s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
          ((p3 - s3)*s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p1 - s1)*x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p2 - s2)*x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p3 - s3)*x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
        (-((p1 - s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          (pow(p1 - s1,2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s1 + x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p1 - s1)*(p2 - s2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          ((p1 - s1)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-((pow(p1 - s1,2)*s1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) - 
      ((p1 - s1)*(p2 - s2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      (p1 - s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + s1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
      ((p1 - s1)*(p3 - s3)*s3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      (pow(p1 - s1,2)*x1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      x1/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p1 - s1)*(p2 - s2)*x2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      ((p1 - s1)*(p3 - s3)*x3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(2, 1) = -(((-(((p1 - s1)*s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
          ((p2 - s2)*s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
          ((p3 - s3)*s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p1 - s1)*x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p2 - s2)*x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p3 - s3)*x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
        (-((p2 - s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
          ((p1 - s1)*(p2 - s2)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
          (pow(p2 - s2,2)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
          (-s2 + x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
          ((p2 - s2)*(p3 - s3)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
      pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2)) + 
   (-(((p1 - s1)*s1*(p2 - s2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) - 
      (pow(p2 - s2,2)*s2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      (p2 - s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + s2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
      ((p2 - s2)*(p3 - s3)*s3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      ((p1 - s1)*(p2 - s2)*x1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      (pow(p2 - s2,2)*x2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      x2/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(p3 - s3)*x3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5))/
    (((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)));

    H_tilde_partial(2, 2) = (-(((p1 - s1)*s1*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)) - 
      ((p2 - s2)*s2*(p3 - s3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      (p3 - s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + s3/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
      (pow(p3 - s3,2)*s3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      ((p1 - s1)*(p3 - s3)*x1)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
      ((p2 - s2)*(p3 - s3)*x2)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
      x3/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + (pow(p3 - s3,2)*x3)/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)
      )/(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
   ((-(((p1 - s1)*s1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) - 
        ((p2 - s2)*s2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) - 
        ((p3 - s3)*s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p1 - s1)*x1)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p2 - s2)*x2)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        ((p3 - s3)*x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)))*
      (-((p3 - s3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2))) + 
        ((p1 - s1)*(p3 - s3)*(-s1 + x1))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) + 
        ((p2 - s2)*(p3 - s3)*(-s2 + x2))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5) - 
        (-s3 + x3)/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
        (pow(p3 - s3,2)*(-s3 + x3))/pow(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2),1.5)))/
    pow(((p1 - s1)*(-s1 + x1))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p2 - s2)*(-s2 + x2))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)) + 
      ((p3 - s3)*(-s3 + x3))/sqrt(pow(p1 - s1,2) + pow(p2 - s2,2) + pow(p3 - s3,2)),2);

 //    if(sid > 4)
 //    {
 //    	cout << "s1 = " << s1 << "\ts2 = " << s2 << "\ts3 = " << s3 << endl;
	// 	cout << "p1 = " << p1 << "\tp2 = " << p2 << "\tp3 = " << p3 << endl;
	// 	cout << "x1 = " << x1 << "\tx2 = " << x2 << "\tx3 = " << x3 << endl;
	//     cout << "H partial = \n" << H_tilde_partial << endl << endl;
	// }

	mat H_tilde = (DELTA.t() * DELTA).i() * DELTA.t() * T.i() * H_tilde_partial; // dh/ds

	return H_tilde;
}
