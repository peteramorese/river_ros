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
// End get parameter values




// Initialize the System
SYSTEM sys(sim);

// Run pickup sensors calibration
sys.calibrate_pickup();

// Run dropoff sensor calibration
sys.calibrate_dropoff();

cin.get();

// Update system parameters
sys.update_params();

// Continuously loop the estimator
sys.loop_estimator();

} // End main()
