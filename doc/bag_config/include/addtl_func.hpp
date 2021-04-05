#include <vector>
#include <string>
#include <fstream>
#include <iostream>
using namespace std;

std::vector<double> get_time_vector(double t_start, double t_stop, double dT);

void print_data(std::vector<std::vector<std::vector<DATA>>> Y);

void plot_ekf(std::vector<double> t, std::vector<EKF> ekf);

std::vector<std::vector<DATA>> read_data(string filename);