#include "SYSTEM.hpp"
using namespace std;

// roslaunch river_ros bag_config_node.launch

int main()
{

ros::VP_string remappings;
remappings.push_back(std::make_pair("", ""));
ros::init(remappings, "calibrator");

// ros::NodeHandle nh;

// vector<string> keys;
// nh.getParamNames(keys);

// for(int i = 0; i < keys.size(); i++)
// {
// 	cout << "Key " << i << " " << keys[i] << endl;
// }

string sw_mode;
ros::param::get("/bag_config_node/software_mode", sw_mode);
// cout << "sw " << ros::param::has("/bag_config_node/software_mode") << endl;

bool sim = false; // Set software mode
if(sw_mode == "Simulation")
{
	sim = true;
}

bool plot_c; bool plot_e;
ros::param::get("/bag_config_node/plot/calibration", plot_c);
ros::param::get("/bag_config_node/plot/estimation", plot_e);

vector<bool> plot = {plot_c, plot_e}; // Set the plot flags

// Initialize the System
SYSTEM system(sim, plot);

int cal_num_mark;
bool run_cal_p;
ros::param::get("/bag_config_node/run_calibration/pickup", run_cal_p);

if(run_cal_p)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	ros::param::get("/bag_config_node/calibrator/pickup/num_markers", cal_num_mark);

	BAG calibrator_pickup(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/pickup/marker_";
		param_str = param_str + to_string(i);
		ros::param::get(param_str, m_i_pos);
		calibrator_pickup.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
	}

	system.assign_bag(calibrator_pickup);

}

system.calibrate_pickup();

bool run_cal_d;
ros::param::get("/bag_config_node/run_calibration/dropoff", run_cal_d);

if(run_cal_d)
{
	cout << "Press Enter to calibrate." << endl;
	cin.get();

	ros::param::get("/bag_config_node/calibrator/dropoff/num_markers", cal_num_mark);

	BAG calibrator_dropoff(cal_num_mark);

	string param_str;
	vector<double> m_i_pos;

	for(int i = 0; i < cal_num_mark; i++)
	{
		param_str = "/bag_config_node/calibrator/dropoff/marker_";
		param_str = param_str + to_string(i);
		ros::param::get(param_str, m_i_pos);
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

}


