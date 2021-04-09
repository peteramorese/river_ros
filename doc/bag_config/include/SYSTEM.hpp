#include "CONSTANTS.hpp"
#include "CORE.hpp"
#include "DATA.hpp"
#include "MARKER.hpp"
#include "BAG.hpp"
#include "SENSOR.hpp"
#include "SENSOR_MIN.hpp"
#include "EKF.hpp"
#include "ERRORS.hpp"

#include "oriDetermine.h"
#include "matrix.h"

#include "matplotlibcpp.h"

#include "ros/ros.h"
#include "river_ros/data_pt.h"
#include "river_ros/data_pkt.h"
#include "river_ros/data_pkg.h"
#include "river_ros/BagConfigPoseArray_msg.h"
#include "river_ros/Observe_srv.h"
#include "geometry_msgs/Pose.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <armadillo>
#include <stdlib.h>
#include <cstdlib>

using namespace std;
using namespace arma;
namespace plt = matplotlibcpp;


class SYSTEM
{
public:
	CONSTANTS cnst; // Object defining system-wide constants
	bool sim; // Flag defining simulation run mode

	SYSTEM(bool s);
	~SYSTEM();
	void set_sim_flag(bool);
	void assign_bag(BAG);
	BAG get_bag();
	void calibrate_pickup();
	void calibrate_dropoff();
	void run_estimator(std::vector<int>);
	void run_estimator_pickup();
	void run_estimator_dropoff();
	vector<vector<vector<DATA>>> get_data(std::vector<int>, string, int);
	void calibrate_callback(const river_ros::data_pkg::ConstPtr&);
	void estimator_callback(const river_ros::data_pkg::ConstPtr&);
	void update_params();
	void loop_estimator();
	// void clear_data();


private:
	CORE core; // Object defining the core frame
	vector<SENSOR> sensors; // Vector defining the network of sensors
	vector<SENSOR_MIN> sensors_min; // Vector defining the network of sensors (minimal)
	BAG bag; // Object defining the cargo bag being estimated
	std::chrono::time_point<std::chrono::system_clock> est_start; // Time that estimation begins
	double stop_est_cov_thrsh; // Threshold to stop estimation
	double stop_est_time; // [s] Threshold to stop estimation
	ros::NodeHandle est_nh;
	river_ros::BagConfigPoseArray_msg bag_config_msg;
	// vector<vector<vector<DATA>>> Y; // Vector to store data in

	void init_default_sensors();
	void calibrate(std::vector<int> s);
	vector<vector<DATA>> read_data(string);
	void set_sensors_min(vector<int>);
	bool observe_srv_callback(river_ros::Observe_srv::Request &req, river_ros::Observe_srv::Response &res);
	void reset_bag_config_msg();
	void upd_bag_config_msg(string);
	void send_bag_config_msg();
	vector<vector<vector<DATA>>> reorg_data(vector<vector<vector<DATA>>>);
};