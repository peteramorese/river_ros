#pragma once
#include<vector>
#include<utility>
#include "matrix.h"

class oriDetermine {
	private:
		double norm3(double* arr_ptr);
		void cross3(double* A, double* B, double* product_ptr);
		matrix<double> q2R(double* v, double theta);
	public:
		oriDetermine(std::vector<std::pair<double[3], int>>& bag_);
		void setData(std::vector<std::pair<double[3], int>>& coords_3Ddata_);
		void setIterations(int Niter_);
		void setAlpha(double alpha_);
		matrix<double> returnQ();


};
