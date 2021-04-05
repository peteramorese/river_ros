#include "CONSTANTS.hpp"
#include "CORE.hpp"
#include "DATA.hpp"
#include "MARKER.hpp"
#include "BAG.hpp"
#include "SENSOR.hpp"
#include "SENSOR_MIN.hpp"
#include "EKF.hpp"

#include "oriDetermine.h"
#include "matrix.h"

#include "matplotlibcpp.h"

#include "ros/ros.h"
#include "river_ros/data_pt.h"
#include "river_ros/data_pkt.h"
#include "river_ros/data_pkg.h"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <armadillo>

using namespace std;
using namespace arma;
namespace plt = matplotlibcpp;


class SYSTEM
{
public:
	bool sim; // Flag defining simulation run mode
	vector<bool> plot = {false, false}; // Flag to plot results {calibration, estimation}

	SYSTEM(bool s = true, vector<bool> p = {false, false});
	~SYSTEM();
	void set_sim_flag(bool);
	void assign_bag(BAG);
	void calibrate_pickup();
	void calibrate_dropoff();
	void run_estimator(std::vector<int>);
	void run_estimator_pickup();
	void run_estimator_dropoff();
	vector<vector<vector<DATA>>> get_data(std::vector<int>, string, int);
	void calibrate_callback(const river_ros::data_pkg::ConstPtr&);
	void estimator_callback(const river_ros::data_pkg::ConstPtr&);
	// void clear_data();


private:
	CORE core; // Object defining the core frame
	CONSTANTS cnst; // Object defining system-wide constants
	vector<SENSOR> sensors; // Vector defining the network of sensors
	vector<SENSOR_MIN> sensors_min; // Vector defining the network of sensors (minimal)
	BAG bag; // Object defining the cargo bag being estimated
	std::chrono::time_point<std::chrono::system_clock> est_start; // Time that estimation begins
	double stop_est_cov_thrsh; // Threshold to stop estimation
	double stop_est_time; // [s] Threshold to stop estimation
	// vector<vector<vector<DATA>>> Y; // Vector to store data in

	void init_default_sensors();
	void calibrate(std::vector<int> s);
	vector<vector<DATA>> read_data(string);
	void set_sensors_min(vector<int>);
	vector<vector<vector<DATA>>> reorg_data(vector<vector<vector<DATA>>>);
};