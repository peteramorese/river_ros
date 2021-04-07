#include "SYSTEM.hpp"

#include <armadillo>
using namespace arma;
using namespace std;


SYSTEM::SYSTEM(bool s, vector<bool> p)
{
	// Initializes the SYSTEM object

	cout << "Initializing System...\t\t" << endl;

	sim = s; // Set the software run mode
	plot = p; // Set the plotting flag

	init_default_sensors(); // Initialize the default sensor network

	set_sim_flag(s); // Propagate the software run mode setting

	double cov_thrsh;
	double time_thrsh;
	string param_str;
	string warn_str;

	param_str = "/bag_config_node/thresholds/covariance";
	cov_thrsh = 0.05;
	if(CheckParam(param_str, 1, cov_thrsh))
	{
		ros::param::get(param_str, cov_thrsh);
	}

	param_str = "/bag_config_node/thresholds/time";
	time_thrsh = 60;
	if(CheckParam(param_str, 1, time_thrsh))
	{
		ros::param::get(param_str, time_thrsh);
	}
		
	stop_est_cov_thrsh = cov_thrsh;
	stop_est_time = time_thrsh;

	cout << "DONE" << endl;
};


SYSTEM::~SYSTEM(){};


void SYSTEM::init_default_sensors()
{
	// Initializes the nominally defined sensor network

	SENSOR S;

	for(int i = 0; i < cnst.m; i++) // For each sensor
	{
		sensors.push_back(S); // Add sensor to the network
		sensors[i].sim = sim; // Set the software run mode
		sensors[i].init_default_sensor(i, core); // Set the default settings
	}
}


void SYSTEM::set_sim_flag(bool s)
{
	// Propagates the software run mode through the SYSTEM object fields
	// 
	// Inputs:
	// 		s - [bool] Flag defining the software run mode
	// 

	sim = s;
	
	for(int i = 0; i < bag.markers.size(); i++)
	{
		bag.markers[i].sim = sim;
	}

	for(int i = 0; i < sensors.size(); i++)
	{
		sensors[i].sim = sim;
	}

	for(int i = 0; i < sensors_min.size(); i++)
	{
		sensors_min[i].sim = sim;
	}
}


void SYSTEM::assign_bag(BAG b)
{
	// Assigns the SYSTEM object's bag field to a user-defined BAG object
	// 
	// Inputs:
	// 		b - [BAG] User-defined bag to estimate
	// 

	bag = b;

	if(sim == true) // If software is running in simulation mode
	{
		for(int i = 0; i < bag.markers.size(); i++) // For each bag marker
		{
			bag.markers[i].position_true = b.markers[i].position; // Assign position_true
		}
	}

	set_sim_flag(sim); // Propagate the sim flag
}


BAG SYSTEM::get_bag()
{
	return bag;
}


void SYSTEM::calibrate_callback(const river_ros::data_pkg::ConstPtr& package)
{
	// cout << "\n\nNEW MESSAGE" << endl;

	// cout << "Package = " << package->pkt.size() << endl;

	int curr_sid;
	DATA y_k;
	vector<DATA> Y_k;
	vector<vector<DATA>> Y;
	EKF e;
	bool run_upd;

	for(int i = 0; i < package->pkt.size(); i++)
	{
		for(int j = 0; j < package->pkt[i].pt.size(); j++)
		{
			y_k.x = package->pkt[i].pt[j].x;
			y_k.y = package->pkt[i].pt[j].y;
			y_k.mid = package->pkt[i].pt[j].mid;
			y_k.sid = package->pkt[i].pt[j].sid;

			curr_sid = y_k.sid;

			Y_k.push_back(y_k);
		}

		Y.push_back(Y_k);

		e = sensors[curr_sid].ekf.back();
		run_upd = false; // Assume the update does not need to be run

		for(int j = 0; j < cnst.n; j++)
		{
			if(2.0*sqrt(e.P(j, j)) > stop_est_cov_thrsh)
			{
				run_upd = true;
			}
		}
		
		if(run_upd)
		{
			sensors[curr_sid].calibrate_sensor(Y, bag, core, cnst);
		}
		else
		{
			sensors[curr_sid].ekf.push_back(e);
		}
	}
}


void SYSTEM::calibrate(std::vector<int> s)
{
	// Utilizes an Extended Kalman Filter (EKF) to calibrate the sensor positions for each sensor in the network
	// 
	// Inputs:
	// 		cal - [BAG] Series of markers with known locations used to calibrate the sensor positions
	// 

	cout << "Calibrating Sensors..." << endl;
	cout << "\n\n\n\n\n";

	est_start = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> current_time;
	std::chrono::duration<double> delta_time;
	bool time_stop = false;

	// ros::VP_string remappings;
	// remappings.push_back(std::make_pair("chatter", "chatter"));
	// ros::init(remappings, "calibrator");

	ros::NodeHandle cal_nh;

	ros::Subscriber cal_sub = cal_nh.subscribe("chatter", 1000, &SYSTEM::calibrate_callback, this);

	bool loop = true;

	EKF e;

	string param_str = "/bag_config_node/verbose";
	bool verbose = true;
	if(CheckParam(param_str, 1, verbose))
	{
		ros::param::get(param_str, verbose);
	}

	while(loop)
	{
		loop = false;

		if(verbose)
		{
			printf("\033[A\033[A\033[A\033[A\033[A");
		}

		for(int i = 0; i < s.size(); i++)
		{
			e = sensors[s[i]].ekf.back();

			if(verbose)
			{
				printf("\33[2KT\r");

				cout << "Sensor " << s[i] << " 2sigma:";
			}

			for(int j = 0; j < cnst.n; j++)
			{
				if(verbose)
				{
					printf("\r");
					if(j == 0)
					{
						cout << "\t\t\tx = ";
					}
					else if(j == 1)
					{
						cout << "\t\t\t\t\t\ty = ";
					}
					else if(j == 2)
					{
						cout << "\t\t\t\t\t\t\t\t\tz = ";
					}
					
					cout << 2.0*sqrt(e.P(j, j));
				}

				if(2.0*sqrt(e.P(j, j)) > stop_est_cov_thrsh)
				{
					loop = true;
				}
			}

			cout << endl;
		}

		current_time = std::chrono::system_clock::now();
		delta_time = current_time - est_start;

		if(delta_time.count() > stop_est_time)
		{
			loop = false;
			time_stop = true;
		}

		ros::spinOnce();

	}

	if(time_stop)
	{
		cout << "The necessary covariance threshold was not met before the maximum estimation time was reached." << endl;
	}
	else
	{
		cout << "Done" << endl;
	}
	// ros::spin();

	for(int i = 0; i < s.size(); i++) // For each sensor in the network
	{
		// cout << "\tCalibrating Sensor " << s[i] << "...\t\t";

		// cout << " ";

		// sensors[s[i]].calibrate_sensor(Y[i], cal, core, cnst); // Calibrate Sensor SID

		// Update ros params
		bool upd_sens = false;
		string param_str = "/bag_config_node/sensor_";
		param_str = param_str + to_string(s[i]) + "/position";
		if(CheckParam(param_str, 1, upd_sens))
		{
			ros::param::get(param_str, upd_sens);
		}

		if(upd_sens)
		{
			vector<double> sens_upd;
			EKF e = sensors[s[i]].ekf.back();
			if(e.x_hat.size() == 3)
			{
				sens_upd = {e.x_hat[0], e.x_hat[1], e.x_hat[2]};
				ros::param::set(param_str, sens_upd);
			}
			else
			{
				string warn_str = "Update for rosparam " + param_str + " has failed.";
				WARNING(warn_str);
			}
		}

		if(plot[0] == true) // If the plot flag is set to true
		{
			sensors[s[i]].plot_ekf(); // Plot the EKF results
			// sensors[i].plot_e_y();
		}

		// cout << "DONE" << endl;
	}

}


void SYSTEM::calibrate_pickup()
{
	// Selects the correct sensors to calibrate (Sensors 0 - 4)
	// 

	string param_str;
	std::vector<int> s;

	for(int i = 0; i < 5; i++)
	{
		s.push_back(i);
	}

	bool run_cal_p;
	run_cal_p = false;

	param_str = "/bag_config_node/run_calibration/pickup";
	if(CheckParam(param_str, 1, run_cal_p))
	{
		ros::param::get(param_str, run_cal_p);
	}

	if(run_cal_p)
	{
		calibrate(s);
	}

	set_sensors_min(s); // Assign the minimal sensor object
}


void SYSTEM::calibrate_dropoff()
{
	// Selects the correct sensors to calibrate (Sensors 5 - 9)
	// 

	string param_str;
	std::vector<int> s;

	for(int i = 0; i < 5; i++)
	{
		s.push_back(5 + i);
	}

	bool run_cal_d;
	run_cal_d = false;
	param_str = "/bag_config_node/run_calibration/dropoff";
	if(CheckParam(param_str, 1, run_cal_d))
	{
		ros::param::get(param_str, run_cal_d);
	}

	if(run_cal_d)
	{
		calibrate(s);
	}

	set_sensors_min(s); // Assign the minimal sensor object
}


void SYSTEM::set_sensors_min(vector<int> s)
{
	// Assign the properties of the minimal sensor network
	// The SENSOR_MIN class is used for bag estimation after calibration has been completed
	// Objects contain only the necessary data for bag estimation
	// 

	SENSOR_MIN s_min;

	for(int i = 0; i < s.size(); i++) // For each sensor defined in the network
	{
		s_min.position = sensors[s[i]].position;
		s_min.point = sensors[s[i]].point;
		s_min.frame = sensors[s[i]].frame;
		s_min.Q_C = sensors[s[i]].Q_C;
		s_min.R = sensors[s[i]].R;
		s_min.dT = sensors[s[i]].dT;
		s_min.sid = sensors[s[i]].sid;
		s_min.sim = sensors[s[i]].sim;

		sensors_min.push_back(s_min);

		// cout << "S" << s[i] << "  " << sensors_min[s[i]].position[0] << "  " << sensors_min[s[i]].position[1] << "  " << sensors_min[s[i]].position[2] << endl;
	}
}


vector<vector<vector<DATA>>> SYSTEM::get_data(std::vector<int> s, string filename, int i_repl)
{
	// Gets sensor data and organizes the data into a multidimensional vector.
	// Function reads the data from files with the name: './data/Y_sim[sid].txt'
	// 
	// Inputs:
	// 		filename - [string] Path and filename to the data file to read
	// 		i_repl - [int] Index of the character in filename to replace and iterate for each sensor

	vector<vector<vector<DATA>>> Y;
	vector<vector<DATA>> Y_sid;

	for(int i = 0; i < s.size(); i++) // For each sensor in the network
	{
		filename[i_repl] = to_string(s[i])[0]; // Set the SID in the filename

		Y_sid = read_data(filename); // Read the data file
		Y.push_back(Y_sid); // Add the data vector to the multidimensional data vector
	}

	return Y;
}


std::vector<std::vector<DATA>> SYSTEM::read_data(string filename)
{
	// Reads the data file and extracts the data to a multidimensional vector
	// 
	// Inputs:
	// 		filename - [string] Path and filename to the data file
	// 
	// Outputs:
	// 		Y_sid - [vector<vector<DATA>>] Vector containing data for all time k for each marker MID
	// 

	std::vector<std::vector<DATA>> Y_sid;
	std::vector<DATA> Y_k;

	std::ifstream file(filename);

	if(file.good())
	{
		string str;
		string substr;
		int cnt;
		DATA y_k;

		vector<int> mids; // Vector containing the Marker IDs of measured markers at time k
		vector<int> mid_meas; // Vector containing the number of times a Marker ID has been measured at time k

		while(std::getline(file, str)) // Read each line of the data file
		{
			stringstream part(str);

			cnt = 0;

			bool clr = false;

			while(part.good()) 
			{
				cnt++;

				getline(part, substr, ','); // Read each comma-separated field of the line

				// Assign correct data fields
				if(cnt == 1) // x measurement
				{
					y_k.x = stod(substr);
				}
				else if(cnt == 2) // y measurement
				{
					y_k.y = stod(substr);
				}
				else if(cnt == 3) // Marker ID mid
				{
					y_k.mid = stoi(substr);

					// This next part of the function determines how many times each marker has been measured at time k
					// If this measurement is the first measurement of marker mid, continue adding data for time k
					// If this measurement is the second measurement of marker mid, push the data (not including this second measurement)
					// and then move to time k+1
					// This prevents multiple measurements of the same marker from being used at the same time k

					bool found = false;
					int i_found;

					for(int i = 0; i < mids.size(); i++) // Loop through the marker IDs that have been measured
					{
						if(mids[i] == y_k.mid) // If the current measurement marker ID is there, flag it
						{
							found = true;
							i_found = i;
						}
					}

					if(found == false) // If the marker ID was not found, add the marker ID to mids
					{
						mids.push_back(y_k.mid);
						mid_meas.push_back(1); // Record that the marker ID has been measured once
					}
					else // If the marker ID was found, increment the measurement count for that marker
					{
						mid_meas[i_found] = mid_meas[i_found] + 1;
						if(mid_meas[i_found] == 2) // If this is the second measurement of marker MID, flag it
						{
							clr =  true;
						}
					}
				}
				else if(cnt == 4) // Sensor ID sid
				{
					y_k.sid = stoi(substr);
				}
			}

			if(clr == true) // If any marker has been measured twice
			{
				Y_sid.push_back(Y_k); // Add the data for time k
				Y_k.clear(); // Clear the time k measurement vector

				for(int i = 0; i < mid_meas.size(); i++) // Loop through the measurement counts
				{
					if(mid_meas[i] > 0) // Decrement the count if it is not equal to zero
					{
						mid_meas[i] = mid_meas[i] - 1;
					}
				}
			}

			// Add DATA measurement to vectors here
			Y_k.push_back(y_k);
		}
	}
	else
	{
		cout << "File: " << filename << " does not exist" << endl;
	}

	return Y_sid;
}


void SYSTEM::estimator_callback(const river_ros::data_pkg::ConstPtr& package)
{

	EKF e;
	DATA y_k;
	vector<DATA> Y_k;
	vector<vector<DATA>> Y;
	bool run_upd;

	for(int cur_mid = 0; cur_mid < bag.markers.size(); cur_mid++)
	{

		run_upd = false; // Assume the update does not need to be run
		e = bag.markers[cur_mid].ekf.back();

		for(int i_n = 0; i_n < cnst.n; i_n++)
		{
			if(2.0*sqrt(e.P(i_n, i_n)) > stop_est_cov_thrsh)
			{
				run_upd = true;
			}
		}

		if(run_upd)
		{
			for(int i = 0; i < package->pkt.size(); i++)
			{
				for(int j = 0; j < package->pkt[i].pt.size(); j++)
				{
					y_k.x = package->pkt[i].pt[j].x;
					y_k.y = package->pkt[i].pt[j].y;
					y_k.mid = package->pkt[i].pt[j].mid;
					y_k.sid = package->pkt[i].pt[j].sid;

					if(y_k.mid == cur_mid)
					{
						Y_k.push_back(y_k);
					}
				}
			}

			if(Y_k.size() > 1)
			{
				string param_str = "/bag_config_node/verbose";
				bool verbose = true;
				if(CheckParam(param_str, 1, verbose))
				{
					ros::param::get(param_str, verbose);
				}
				if(verbose)
				{
					cout << "Estimating Marker " << cur_mid << endl;
				}

				Y.push_back(Y_k);
				bag.markers[cur_mid].estimate_pos(Y, sensors_min, core, cnst);

				Y.clear();
			}
		}

		Y_k.clear();
	}

}


void SYSTEM::run_estimator(std::vector<int> s)
{
	// Uses Pixy sensor data to run an Extended Kalman Filter (EKF) for each marker that contains measurements
	// 
	// Inputs:
	// 		s - [vector<int>] Vector containing the Sensor IDs to run the estimator with
	// 

	cout << "Running Position Estimator..." << endl;

	// string filename = "../data/Y_sim__test.txt";
	// vector<vector<vector<DATA>>> Y = get_data(s, filename, 13);

	// string filename = "../data/box_cam__final.txt";
	// vector<vector<vector<DATA>>> Y = get_data(s, filename, 15);

	// vector<vector<vector<DATA>>> Y_reorg;

	// Y_reorg = reorg_data(Y); // Reorganize the data vector

	// for(int mid = 0; mid < Y_reorg.size(); mid++)
	// {
	// 	if(Y_reorg[mid].size() > 0)
	// 	{
	// 		cout << "\tEstimating Marker " << mid << "... ";

	// 		bag.markers[mid].estimate_pos(Y_reorg[mid], sensors_min, core, cnst); // Estimate the position of Marker MID

	// 		if(plot[1] == true && bag.markers[mid].updated == true) // If the plot flag is set to true
	// 		{
	// 			bag.markers[mid].plot_ekf(); 
	// 			// bag.markers[mid].plot_e_y(); 
	// 		}

	// 		cout << "DONE" << endl;
	// 	}
	// }

	est_start = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> current_time;
	std::chrono::duration<double> time_elapsed;
	bool time_stop = false;

	ros::NodeHandle est_nh;

	// Create ros subscriber
	ros::Subscriber est_sub = est_nh.subscribe("chatter", 1000, &SYSTEM::estimator_callback, this);

	bool loop = true;
	EKF e;

	// Estimate continuously until an exit condition is met
	while(loop)
	{
		loop = false; // Assume an exit condition is met, set loop = true if not

		for(int i = 0; i < bag.markers.size(); i++) // For each marker on the bag
		{
			e = bag.markers[i].ekf.back(); // Get the latest state estimate

			for(int j = 0; j < cnst.n; j++) // For each state being estimated (position in this case)
			{

				// Continue looping if any of the 2sigma bounds are greater than the set threshold
				if(2.0*sqrt(e.P(j, j)) > stop_est_cov_thrsh)
				{
					loop = true;
				}
			}
		}

		current_time = std::chrono::system_clock::now();
		time_elapsed = current_time - est_start;

		// If the time elapsed is greater than the set threshold stop looping
		if(time_elapsed.count() > stop_est_time)
		{
			loop = false;
			time_stop = true;
		}

		ros::spinOnce();
	}

	if(time_stop)
	{
		cout << "The necessary covariance threshold was not met before the maximum estimation time was reached." << endl;
	}
	else
	{
		cout << "Done" << endl;
	}

	cout << "Running Orientation Estimator..." << endl;

	int upd_markers = 0;

	for(int i = 0; i < bag.markers.size(); i++)
	{
		if(bag.markers[i].updated == true)
		{
			upd_markers++;
		}
	}

	if(upd_markers > 2)
	{
		bag.bag_found = true;
		bag.estimate_ori();
	}
	else if(upd_markers < 3)
	{
		cout << "Bag has not been found. Only " << upd_markers << " markers have been updated." << endl;
	}

	cout << "DONE" << endl;
}


void SYSTEM::run_estimator_pickup()
{
	// Function to run the estimation algorithm at the pickup location

	std::vector<int> s;

	for(int i = 0; i < 5; i++)
	{
		s.push_back(i);
	}

	run_estimator(s);
}


void SYSTEM::run_estimator_dropoff()
{
	// Function to run the estimation algorithm at the pickup location

	std::vector<int> s;

	for(int i = 5; i < 10; i++)
	{
		s.push_back(i);
	}

	run_estimator(s);
}


vector<vector<vector<DATA>>> SYSTEM::reorg_data(vector<vector<vector<DATA>>> Y)
{
	// Function to reorganize the data vector Y from Y[sid][k][mid] to Y[mid][k][sid]
	// 
	// Output:
	// 		Y_reorg - [vector<vector<vector<DATA>>>] Reorganized data vector
	// 

	vector<vector<vector<DATA>>> Y_reorg;
	vector<vector<DATA>> Y_k;
	vector<DATA> Y_sid;

	for(int mid = 0; mid < bag.markers.size(); mid++) // Loop through all possible marker IDs
	{
		for(int k = 0; k < Y[0].size(); k++) // Loop through all time steps k
		{
			for(int sid = 0; sid < Y.size(); sid++) // Loop through all sensors
			{
				if(Y[sid].size() > 0)
				{
					for(int m = 0; m < Y[sid][k].size(); m++) // Loop through all DATA objects
					{
						if(Y[sid][k][m].mid == mid) // If the measurement is for Marker mid, add it to the vector
						{
							Y_sid.push_back(Y[sid][k][m]);
							break;
						}
					}
				}
			}
			if(Y_sid.size() > 0) // Only add measurements if there are measurements to add
			{
				Y_k.push_back(Y_sid); // Add all measurements of mid at time k
				Y_sid.clear(); // Clear the vector for the next time k
			}
		}
		Y_reorg.push_back(Y_k); // Add all measurements of mid for all time k
		Y_k.clear(); // Clear the vector for the next marker ID
	}

	return Y_reorg;
}
