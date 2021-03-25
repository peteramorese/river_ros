#include "MARKER.hpp"

#include "EKF.hpp"

#include "matplotlibcpp.h"

#include <armadillo>
using namespace arma;
using namespace std;
namespace plt = matplotlibcpp;


MARKER::MARKER()
{
	// Initializer

	position = zeros<vec>(3);
	mid = 0;
	plate = 0;

	init_ekf(); // Initialize the EKF results
}


MARKER::MARKER(vec p, int m, int pl)
{
	// Initializer

	position = p;
	mid = m;
	plate = pl;

	init_ekf(); // Initialize the EKF results
}


MARKER::~MARKER(){};


void MARKER::init_ekf()
{
	// Initialize the EKF results

	EKF e;
	e.x_hat = position;
	e.P = {{pow(5, 2), 0, 0},
			{0, pow(1.195, 2), 0},
			{0, 0, pow(1.195, 2)}};

	ekf.push_back(e);
}


void MARKER::estimate_pos(vector<vector<DATA>> Y, vector<SENSOR_MIN> sensors, CORE core, CONSTANTS cnst)
{
	// Takes each collection of data for time k and runs an EKF update
	// Adds the EKF update to ekf
	// 
	// Inputs:
	// 		Y - [vector<vector<vector<DATA>>>] Multidimensional data vector containing all data of
	// 			marker mid from each sensor for all time k. Y[mid][k][sid]
	// 		core - [CORE] Object defining the Core Frame
	// 		cnst - [CONSTANTS] Object defining system-wide constants
	// 

	EKF e;
	vector<DATA> y_k;

	for(int k = 0; k < Y.size(); k++)
	{
		if(Y[k].size() > 1) // Only perform EKF update if the marker has multiple sensor measurements
		{
			y_k = Y[k];
			e = ekf_update(ekf[ekf.size()-1], sensors, y_k, core, cnst); // Perform an EKF update
			ekf.push_back(e);

			updated = true; // Switch flag to indicate the marker has been estimated
		}
	}
}


EKF MARKER::ekf_update(EKF e, vector<SENSOR_MIN> sensors, vector<DATA> y_k, CORE core, CONSTANTS cnst)
{
	// Completes a single EKF update given the previous step e and the data y_k
	// 
	// Inputs:
	// 		e - [EKF] Object containing the EKF data from the previous update
	// 		y_k - [vector<DATA>] Vector containing the data for marker mid at time k
	// 		core - [CORE] Object defining the Core Frame
	// 		cnst - [CONSTANTS] Object defining system-wide constants
	// 
	// Output:
	// 		e_kp1 - [EKF] Updated EKF step
	// 

	EKF e_kp1;
	int num_meas = 0; // Number of measurements
	vector<int> sid_valid;

	for(int i = 0; i < y_k.size(); i++) // Check each sensor for a valid measurement
	{
		if(isnan(y_k[i].x) == 0)
		{
			num_meas++; // Measurement is valid
			sid_valid.push_back(y_k[i].sid);
		}
	}

	if(num_meas > 1) // Only perform an EKF update if there are multiple measurements of the marker mid
	{
		tuple<double, double> yhat_tup;
		vec yhat(cnst.p*num_meas);
		vec y(cnst.p*num_meas);
		mat H_tilde;
		mat R_tmp;
		mat R_blk;

		for(int i = 0; i < num_meas; i++) // For each measurement
		{
			yhat_tup = get_yhat(sensors[sid_valid[i]], core);

			yhat[2*i] = std::get<0>(yhat_tup); // Store yhat in a stacked vector
			yhat[2*i+1] = std::get<1>(yhat_tup);

			y[2*i] = y_k[i].x; // Store real measurements in a stacked vector
			y[2*i+1] = y_k[i].y;

			mat H = get_H_tilde(sensors[sid_valid[i]], core);
			H_tilde = std::move(arma::join_cols(H_tilde, H));

			R_tmp = std::move(arma::join_rows(zeros(2, R_blk.n_cols), sensors[sid_valid[i]].R));
			R_blk = std::move(arma::join_rows(R_blk, zeros(R_blk.n_rows, 2)));
			R_blk = std::move(arma::join_cols(R_blk, R_tmp));
		}

		e_kp1.e_y = y - yhat; // Measurement Innovation

		// Kalman gain matrix
		mat K_tilde = e.P * H_tilde.t() * (H_tilde * e.P * H_tilde.t() + R_blk).i();

		e_kp1.x_hat = e.x_hat + K_tilde * e_kp1.e_y; // New state estimate

		mat I = eye(cnst.n, cnst.n);

		e_kp1.P = (I - K_tilde * H_tilde) * e.P; // New covariance matrix
	}
	else // No measurement update
	{
		e_kp1.x_hat = e.x_hat;
		e_kp1.P = e.P;
	}

	if(sim == true) // If software is running in simulation mode
	{
		e_kp1.e_x = e_kp1.x_hat - position_true; // Estimation error
	}

	position = e_kp1.x_hat; // Update position with state estimate

	return e_kp1;
}


tuple<double, double> MARKER::get_yhat(SENSOR_MIN s, CORE core)
{
	// Simulates the measurement of Marker mid by Sensor sid using the nonlinear measurement model
	// 
	// Inputs:
	// 		s - [SENSOR_MIN] Sensor to simulate the measurement from
	// 		core - [CORE] Object defining the Core Frame
	// 		sim - [bool] Flag defining the software run mode
	// 
	// Output:
	// 		<x, y> - [tuple<double, double>] Simulated measurement
	// 

	tuple<double, double> yhat;

	double x_max = s.x_max;
	double y_max = s.y_max;
	double gamma_rad = s.gamma * M_PI / 180.0; // [rad] Horizontal FOV half-angle
	double theta_rad = s.theta * M_PI / 180.0; // [rad] Vertical FOV half-angle

	mat T = {{tan(gamma_rad), 0, 0},
			{0, tan(theta_rad), 0},
			{0, 0, 1}};

	mat DELTA = {{2.0/x_max, 0},
			{0, 2.0/y_max},
			{0, 0}};

	vec ivec = {1.0, 1.0, -1.0};

	vec V_S = s.frame.t() * core.frame.t() * (position - s.position);
	double a = V_S[2];

	mat H = (1.0/a) * (DELTA.t() * DELTA).i() * DELTA.t() * T.i() * s.frame.t() * core.frame.t();
	mat G = -(1.0/a) * (DELTA.t() * DELTA).i() * DELTA.t() * T.i() * s.frame.t() * core.frame.t() * s.position +
		(DELTA.t() * DELTA).i() * DELTA.t() * ivec;

	vec yhat_vec = H * position + G;

	std::get<0>(yhat) = yhat_vec[0];
	std::get<1>(yhat) = yhat_vec[1];

	return yhat;
}


mat MARKER::get_H_tilde(SENSOR_MIN s, CORE core)
{
	// Computes the Jacobian dh/dx = H_tilde to be used in the EKF update
	// 
	// Inputs:
	// 		s - [SENSOR_MIN] Sensor to simulate the measurement from
	// 		core - [CORE] Object defining the Core Frame
	// 
	// Output:
	// 		H_tilde - [mat] Jacobian dh/dx
	// 

	vec x = position;
	vec s_pos = s.position;
	double x_max = s.x_max;
	double y_max = s.y_max;
	double gamma_rad = s.gamma * M_PI / 180.0; // [rad] Horizontal FOV half-angle
	double theta_rad = s.theta * M_PI / 180.0; // [rad] Vertical FOV half-angle

	mat T = {{tan(gamma_rad), 0, 0},
			{0, tan(theta_rad), 0},
			{0, 0, 1}};

	mat DELTA = {{2.0/x_max, 0},
			{0, 2.0/y_max},
			{0, 0}};

	vec ivec = {1.0, 1.0, -1.0};

	vec z = s.frame.row(2).t();

	rowvec d1_a_dx = {-z[0]*(z[0]*(x[0] - s_pos[0]) + z[1]*(x[1] - s_pos[1]) + z[2]*(x[2] - s_pos[2])),
				-z[1]*(z[0]*(x[0] - s_pos[0]) + z[1]*(x[1] - s_pos[1]) + z[2]*(x[2] - s_pos[2])),
				-z[2]*(z[0]*(x[0] - s_pos[0]) + z[1]*(x[1] - s_pos[1]) + z[2]*(x[2] - s_pos[2]))};

	vec V_S1 = s.frame.t() * core.frame.t() * (x - s_pos);
	double a = V_S1[2];

	mat H_tilde = (1.0/a)*(DELTA.t() * DELTA).i() * DELTA.t() * T.i() * s.frame.t() * core.frame.t() +
		(DELTA.t() * DELTA).i() * DELTA.t() * T.i() * s.frame.t() * core.frame.t() * x * d1_a_dx;

	return H_tilde;
}


void MARKER::plot_ekf()
{
	// Plot the results from the EKF

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
		if(sim == true) // If running in simulation mode
		{
			x.push_back(ekf[i].e_x(0)); // Plot the estimation errors
			y.push_back(ekf[i].e_x(1));
			z.push_back(ekf[i].e_x(2));

			sigx.push_back(2.0*sqrt(ekf[i].P(0, 0))); // 2 Sigma
			sigy.push_back(2.0*sqrt(ekf[i].P(1, 1)));
			sigz.push_back(2.0*sqrt(ekf[i].P(2, 2)));
			nsigx.push_back(-2.0*sqrt(ekf[i].P(0, 0)));
			nsigy.push_back(-2.0*sqrt(ekf[i].P(1, 1)));
			nsigz.push_back(-2.0*sqrt(ekf[i].P(2, 2)));
		}
		else // If not running in simulation mode
		{
			x.push_back(ekf[i].x_hat(0)); // Plot the state estimates
			y.push_back(ekf[i].x_hat(1));
			z.push_back(ekf[i].x_hat(2));

			sigx.push_back(ekf[i].x_hat(0) + 2.0*sqrt(ekf[i].P(0, 0))); // 2 Sigma bounds + state estimate
			sigy.push_back(ekf[i].x_hat(1) + 2.0*sqrt(ekf[i].P(1, 1)));
			sigz.push_back(ekf[i].x_hat(2) + 2.0*sqrt(ekf[i].P(2, 2)));
			nsigx.push_back(ekf[i].x_hat(0) - 2.0*sqrt(ekf[i].P(0, 0)));
			nsigy.push_back(ekf[i].x_hat(1) - 2.0*sqrt(ekf[i].P(1, 1)));
			nsigz.push_back(ekf[i].x_hat(2) - 2.0*sqrt(ekf[i].P(2, 2)));
		}


	}

	std::stringstream ttl;

	if(sim == true)
	{
		ttl << "Estimation Error vs No. of Measurement Updates - Marker " << mid << endl;
	}
	else
	{
		ttl << "Estimation vs No. of Measurement Updates - Marker " << mid << endl;
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
			plt::ylim(x.back() + 5*(nsigx.back() - x.back()), x.back() + 5*(sigx.back() - x.back()));
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
			plt::ylim(y.back() + 5*(nsigy.back() - y.back()), y.back() + 5*(sigy.back() - y.back()));
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
			plt::ylim(z.back() + 5*(nsigz.back() - z.back()), z.back() + 5*(sigz.back() - z.back()));
		}
	plt::show();
}


void MARKER::plot_e_y()
{
	// Plot the measurement innovation results from the calibration Extended Kalman Filter

	std::vector<double> x;
	std::vector<double> y;

	std::vector<double> sigx;
	std::vector<double> sigy;

	for(int i = 1; i < ekf.size(); i++)
	{
		x.push_back(ekf[i].e_y(0));
		y.push_back(ekf[i].e_y(1));
	}

	std::stringstream ttl;
	ttl << "Measurement Innovation vs Measurement Updates - Marker " << mid << endl;

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

}
