#include "SYSTEM.hpp"
using namespace std;

// roslaunch river_ros bag_config_node.launch

int main(int argc, char **argv)
{

ros::init(argc, argv, "estimator");

// Get parameter values
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
// End get parameter values




// Initialize the System
SYSTEM sys(sim, plot);

// Run pickup sensors calibration
sys.calibrate_pickup();

// Run dropoff sensor calibration
sys.calibrate_dropoff();


// int set_mark = 4;
// ros::param::set("/bag_config_node/calibrator/pickup/num_markers", set_mark);
// string cmd_str = "rosparam dump ~/catkin_ws/src/river_ros/config/test.yaml";
// const char *command = cmd_str.c_str();
// system(command);

cout << "ros::ok() = " << ros::ok() << endl;

// while(ros::ok())
// {

	// Run the estimator on the pickup location
	sys.run_estimator_pickup();

	// Run the estimator on the dropoff location
	sys.run_estimator_dropoff();
	
// }

// for(int i = 0; i < bag.markers.size(); i++)
// {
// 	cout << "Marker " << bag.markers[i].mid << " Position = " << bag.markers[i].position[0] << "  " << bag.markers[i].position[1] << "  " << bag.markers[i].position[2] << endl;
// }

} // End main()
