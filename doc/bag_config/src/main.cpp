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

cout << "HELLO" << endl;

string sw_mode;
ros::param::get("/bag_config_node/software_mode", sw_mode);
// cout << "sw " << ros::param::has("/bag_config_node/software_mode") << endl;

bool sim = false; // Set software mode
if(sw_mode == "Simulation")
{
	sim = true;
}
else
{
	sim = false;
}

cout << "Hello 4" << endl;

bool plot_c; bool plot_e;
ros::param::get("/bag_config_node/plot/calibration", plot_c);
ros::param::get("/bag_config_node/plot/estimation", plot_e);

cout << "Hello 5" << endl;

vector<bool> plot = {plot_c, plot_e}; // Set the plot flags

// double x;
// ros::param::get("/bag_config_node/bag_def/num_markers", x);
// cout << "XX = " << x << endl;

// Initialize the System
SYSTEM system(sim, plot);

int cal_num_mark;
bool run_cal_p;
ros::param::get("/bag_config_node/run_calibration/pickup", run_cal_p);

cout << "Hello 6 " << endl;

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

cout << "HELLO 2" << endl;

system.calibrate_pickup();

cout << "HELLO 3" << endl;

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

// Main Menu
// menu = true;
// bool pickup_cal = false;
// bool dropoff_cal = false;

// while(menu == true)
// {
// 	cout << "\n\n\t\tMENU\t\t" << endl;
// 	cout << "-------------------------------------------" << endl;
// 	cout << "1) Calibrate Pickup Sensors" << endl;
// 	cout << "2) Calibrate Dropoff Sensors" << endl;
// 	if(pickup_cal == true) cout << "3) Scan Pickup" << endl;
// 	if(dropoff_cal == true) cout << "4) Scan Dropoff" << endl;
// 	cout << "0) Exit\n" << endl;

// 	cin >> opt;

// 	cout << endl;

// 	switch(opt)
// 	{
// 		case 1 :
// 		{
// 			// Initialize a bag to calibrate the sensors containing 3 markers
// 			// Marker IDs defined here MUST MATCH THE MARKER IDS OUTPUT BY THE SENSORS
// 			// BAG calibrator_p(3);
// 			// calibrator_p.markers[0].position = {0, 0.1, 0.1};
// 			// calibrator_p.markers[1].position = {0.05, -0.05, 0.1};
// 			// calibrator_p.markers[2].position = {0.1, -0.05, 0};

// 			BAG calibrator_p(4);
// 			calibrator_p.markers[0].position = {0, 0, 0};
// 			// 0 = 24
// 			// 1 = 12
// 			// 2 = 14
// 			// 3 = 15
// 			// 
// 			calibrator_p.markers[1].position = {-0.0301625, -0.117475, 0};
// 			calibrator_p.markers[2].position = {0.1476375, 0.098425, 0};
// 			calibrator_p.markers[3].position = {-0.111125, 0.073025, 0};

// 			system.assign_bag(calibrator_p);

// 			// Calibrate sensors 0 - 4
// 			system.calibrate_pickup(calibrator_p);

// 			pickup_cal = true;

// 			break;
// 		}
// 		case 2 :
// 		{
// 			// Initialize a bag to calibrate the dropoff sensors
// 			// Marker IDs defined here MUST MATCH THE MARKER IDS OUTPUT BY THE SENSORS
// 			BAG calibrator_d(6);
// 			calibrator_d.markers[3].position = {2.25, 0.85, -0.80};
// 			calibrator_d.markers[4].position = {2.50, 0.90, -0.8};
// 			calibrator_d.markers[5].position = {2.30, 0.75, -0.75};

// 			// Calibrate sensors 5 - 9
// 			system.calibrate_dropoff(calibrator_d);	

// 			dropoff_cal = true;

// 			break;
// 		}
// 		case 3:
// 		{
// 			BAG bag1(6);
// 			system.assign_bag(bag1);

// 			system.run_estimator_pickup();

// 			break;
// 		}
// 		case 4 :
// 		{
// 			BAG bag2(6);
// 			system.assign_bag(bag2);

// 			system.run_estimator_dropoff();
			
// 			break;
// 		}
// 		case 0 :
// 		{
// 			cout << "Exiting..." << endl;
// 			menu = false;
// 			break;
// 		}
// 		default :
// 		{
// 			cout << "Invalid Selection" << endl;
// 		}
// 	}
// }




// Calibrate the Sensor System
	// system.calibrate(calibrator);

// Initialize the new cargo bag
	// BAG bag1(6);
	// system.assign_bag(bag1);

// Run the bag estimator
	// system.run_estimator();

}


