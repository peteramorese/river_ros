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
if(ros::param::has(param_str))
{
	ros::param::get(param_str, sw_mode);
}
else
{
	sw_mode = "Flight";
	warn_str << "WARNING: The rosparam " << param_str << " does not exist. Defaulting " << param_str << " to " << sw_mode << ".";
	WARNING(warn_str.str());
}

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
if(ros::param::has(param_str))
{
	ros::param::get(param_str, plot_c);
}
else
{
	plot_c = false;
	warn_str << "WARNING: The rosparam " << param_str << " does not exist. Defaulting " << param_str << " to " << plot_c << ".";
	WARNING(warn_str.str());
}

// Plot Estimation Results
bool plot_e;
param_str = "/bag_config_node/plot/estimation";
if(ros::param::has(param_str))
{
	ros::param::get(param_str, plot_e);
}
else
{
	plot_e = false;
	warn_str << "WARNING: The rosparam " << param_str << " does not exist. Defaulting " << param_str << " to " << plot_e << ".";
	WARNING(warn_str.str());
}

vector<bool> plot = {plot_c, plot_e}; // Set the plot flags

// Initialize the System
SYSTEM system(sim, plot);

int cal_num_mark;
bool run_cal_p;
param_str = "/bag_config_node/run_calibration/pickup";
if(ros::param::has(param_str))
{
	ros::param::get(param_str, run_cal_p);
}
else
{
	run_cal_p = false;
	warn_str << "WARNING: The rosparam " << param_str << " does not exist. Defaulting " << param_str << " to " << run_cal_p << ".";
	WARNING(warn_str.str());
}

if(run_cal_p)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	param_str = "/bag_config_node/calibrator/pickup/num_markers";
	if(ros::param::has(param_str))
	{
		ros::param::get(param_str, cal_num_mark);
	}
	else
	{
		warn_str << "ERROR: The rosparam " << param_str << " does not exist.";
		ERROR(warn_str.str());
	}

	BAG calibrator_pickup(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/pickup/marker_";
		param_str = param_str + to_string(i);
		if(ros::param::has(param_str))
		{
			ros::param::get(param_str, m_i_pos);
		}
		else
		{
			warn_str << "ERROR: The rosparam " << param_str << " does not exist.";
			ERROR(warn_str.str());
		}

		calibrator_pickup.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
	}

	system.assign_bag(calibrator_pickup);

}

system.calibrate_pickup();

bool run_cal_d;
param_str = "/bag_config_node/run_calibration/dropoff";
if(ros::param::has(param_str))
{
	ros::param::get(param_str, run_cal_d);
}
else
{
	run_cal_d = false;
	warn_str << "WARNING: The rosparam " << param_str << " does not exist. Defaulting " << param_str << " to " << plot_e << ".";
	WARNING(warn_str.str());
}


if(run_cal_d)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	param_str = "/bag_config_node/calibrator/dropoff/num_markers";
	if(ros::param::has(param_str))
	{
		ros::param::get(param_str, cal_num_mark);
	}
	else
	{
		warn_str << "ERROR: The rosparam " << param_str << " does not exist.";
		ERROR(warn_str.str());
	}

	BAG calibrator_dropoff(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/dropoff/marker_";
		param_str = param_str + to_string(i);
		if(ros::param::has(param_str))
		{
			ros::param::get(param_str, m_i_pos);
		}
		else
		{
			warn_str << "ERROR: The rosparam " << param_str << " does not exist.";
			ERROR(warn_str.str());
		}

		calibrator_dropoff.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
	}

	system.assign_bag(calibrator_dropoff);

}

system.calibrate_dropoff();

BAG bag;

for(int i = 0; i < bag.markers.size(); i++)
{
	cout << "Marker " << bag.markers[i].mid << " Position = " << bag.markers[i].position[0] << "  " << bag.markers[i].position[1] << "  " << bag.markers[i].position[2] << endl;
}

} // End main()