#include "SYSTEM.hpp"
using namespace std;

// roslaunch river_ros bag_config_node.launch

int main(int argc, char **argv)
{

ros::init(argc, argv, "estimator");

// ros::NodeHandle nh;

// vector<string> keys;
// nh.getParamNames(keys);

// for(int i = 0; i < keys.size(); i++)
// {
// 	cout << "Key " << i << " " << keys[i] << endl;
// }
string param_str;
stringstream warn_str;

// Software Mode
string sw_mode;
bool sim;
param_str = "/bag_config_node/software_mode";
sw_mode = "Flight";
CheckParam(param_str, 1, sw_mode);
ros::param::get(param_str, sw_mode);

if(sw_mode == "Simulation")
{
	sim = true;
}
else
{
	sim = false;
}

// Plot Calibration Results
bool plot_c;
param_str = "/bag_config_node/plot/calibration";
plot_c = false;
if(CheckParam(param_str, 1, plot_c))
{
	ros::param::get(param_str, plot_c);
}

// Plot Estimation Results
bool plot_e;
param_str = "/bag_config_node/plot/estimation";
plot_e = false;
if(CheckParam(param_str, 1, plot_e))
{
	ros::param::get(param_str, plot_e);
}

vector<bool> plot = {plot_c, plot_e}; // Set the plot flags

// Initialize the System
SYSTEM sys(sim, plot);

// Calibration Markers
int cal_num_mark;
bool run_cal_p;
param_str = "/bag_config_node/run_calibration/pickup";
run_cal_p = false;
if(CheckParam(param_str, 1, run_cal_p))
{
	ros::param::get(param_str, run_cal_p);
}

if(run_cal_p)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	param_str = "/bag_config_node/calibrator/pickup/num_markers";
	if(CheckParam(param_str, 2))
	{
		ros::param::get(param_str, cal_num_mark);
	}
	
	BAG calibrator_pickup(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/pickup/marker_";
		param_str = param_str + to_string(i);
		if(CheckParam(param_str, 2) && CheckParamSize(param_str, sys.cnst.n))
		{
			ros::param::get(param_str, m_i_pos);
			calibrator_pickup.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
		}
	}

	sys.assign_bag(calibrator_pickup);

}

// Call calibrate_pickup() so that sensors_min get assigned.
// Check is done in the function on whether to calibrate
sys.calibrate_pickup();

bool run_cal_d;
param_str = "/bag_config_node/run_calibration/dropoff";
run_cal_d = false;
if(CheckParam(param_str, 1, run_cal_d))
{
	ros::param::get(param_str, run_cal_d);
}

if(run_cal_d)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	param_str = "/bag_config_node/calibrator/dropoff/num_markers";
	if(CheckParam(param_str, 2))
	{
		ros::param::get(param_str, cal_num_mark);
	}

	BAG calibrator_dropoff(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/dropoff/marker_";
		param_str = param_str + to_string(i);
		if(CheckParam(param_str, 2) && CheckParamSize(param_str, sys.cnst.n))
		{
			ros::param::get(param_str, m_i_pos);
			calibrator_dropoff.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
		}
	}

	sys.assign_bag(calibrator_dropoff);

}

// int set_mark = 4;
// ros::param::set("/bag_config_node/calibrator/pickup/num_markers", set_mark);
// string cmd_str = "rosparam dump ~/catkin_ws/src/river_ros/config/test.yaml";
// const char *command = cmd_str.c_str();
// system(command);

// Call calibrate_dropoff() so that sensors_min get assigned.
// Check is done in the function on whether to calibrate
sys.calibrate_dropoff();

cout << "Press enter to continue to estimation." << endl;
cin.get();

//while(ros::ok())
//{
	BAG bag;
	sys.assign_bag(bag);

	cout << "Estimating pickup location." << endl;
	sys.run_estimator_pickup();
	cout << "Done estimating pickup location." << endl;

	cout << "Send message here." << endl;

	cout << "Estimating dropoff location." << endl;
	sys.run_estimator_dropoff();
	cout << "Done estimating dropoff location." << endl;

//}

// for(int i = 0; i < bag.markers.size(); i++)
// {
// 	cout << "Marker " << bag.markers[i].mid << " Position = " << bag.markers[i].position[0] << "  " << bag.markers[i].position[1] << "  " << bag.markers[i].position[2] << endl;
// }

} // End main()
