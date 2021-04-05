#ifndef EKF_H
#define EKF_H

#include <armadillo>
using namespace arma;
using namespace std;


struct EKF
{
	
	vec x_hat; // State estimate
	mat P; // Covariance matrix
	vec e_x; // State estimation error
	vec e_y; // Measurement innovation
	double eps_x; // NEES Statistic
	double eps_y; // NIS Statistic
};

#endif /* EKF_H */