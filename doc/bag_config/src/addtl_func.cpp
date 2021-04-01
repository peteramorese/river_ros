// Document containing additional functions for position estimation
#include "DATA.hpp"
#include "EKF.hpp"
#include "matplotlibcpp.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;
namespace plt = matplotlibcpp;


std::vector<double> get_time_vector(double t_start, double t_stop, double dT)
{
	vector<double> t;

	int steps = (t_stop - t_start)/dT;

	t.push_back(t_start);

	for(int i = 0; i < steps; i++)
	{
		double t_k = t[i] + dT;
		t.push_back(t_k);
	}

	return t;
}

void print_data(std::vector<std::vector<std::vector<DATA>>> Y)
{
	// Print simulated data
	// cout << "Y.size = " << Y.size() << endl;
	// cout << "Y[0].size = " << Y[0].size() << endl;
	// cout << "Y[0][0].size = " << Y[0][0].size() << endl;

	std::vector<double> Yx_plt;

	for(int sid = 0; sid < Y.size(); sid++)
	{
		// std::cout << "sid = " << sid;

		for(int k = 0; k < Y[sid].size(); k++)
		{
			// std::cout << " k = " << k << std::endl;
			for(int mid = 0; mid < Y[sid][k].size(); mid++)
			{
				// std::cout << "    x = " << Y[sid][k][mid].x << "    y = " << Y[sid][k][mid].y << "    mid = " << Y[sid][k][mid].mid << "    sid = " << Y[sid][k][mid].sid << endl;

				if(sid == 0 && mid == 0)
				{
					Yx_plt.push_back(Y[sid][k][mid].x);
				}
			}
		}
		std::cout << std::endl;
	}

	// Plot simulated data
	plt::plot(Yx_plt, "o-");
	plt::show();

}


