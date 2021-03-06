#include "SYSTEM.hpp"

#include <armadillo>
using namespace arma;
using namespace std;


SYSTEM::SYSTEM(bool s)
{
	// Initializes the SYSTEM object

	cout << "Initializing System...\t\t" << endl;

	sim = s; // Set the software run mode

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

	reset_bag_config_msg();

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
	// cout << "Calibrate Callback" << endl;
	// cout << "\n\nNEW MESSAGE" << endl;

	// cout << "Package = " << package->pkt.size() << endl;

	int curr_sid;
	DATA y_k;
	vector<DATA> Y_k;
	vector<vector<DATA>> Y;
	EKF e;
	bool run_upd;

	if(package->stamp != prev_msg_time)
	{
		// cout << "New Package    " << endl;
		for(int i = 0; i < package->pkt.size(); i++)
		{
			// cout << "New Packet " << i << "   " << endl;
			for(int j = 0; j < package->pkt[i].pt.size(); j++)
			{
				// cout << "New Point " << j << "   ";
				y_k.x = package->pkt[i].pt[j].x;
				y_k.y = package->pkt[i].pt[j].y;
				y_k.mid = package->pkt[i].pt[j].mid;
				y_k.sid = package->pkt[i].pt[j].sid;

				// cout << y_k.x << "   " << y_k.y << "   " << y_k.mid << "   " << y_k.sid << endl;
				curr_sid = y_k.sid;

				Y_k.push_back(y_k);
			}

			Y.push_back(Y_k);
			Y_k.clear();

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
				// cout << "Calibrate Sensor " << curr_sid << endl;
				sensors[curr_sid].calibrate_sensor(Y, bag, core, cnst);
			}
			else
			{
				sensors[curr_sid].ekf.push_back(e);
			}
			Y.clear();
		}

		prev_msg_time = package->stamp;
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
	// cout << "\n\n\n\n\n";

	est_start = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> current_time;
	std::chrono::duration<double> delta_time;
	bool time_stop = false;

	// ros::NodeHandle cal_nh;

	ros::Subscriber cal_sub = est_nh.subscribe("arduino/data_read", 1000, &SYSTEM::calibrate_callback, this);

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
			// printf("\033[A\033[A\033[A\033[A\033[A");
		}

		for(int i = 0; i < s.size(); i++)
		{
			e = sensors[s[i]].ekf.back();

			if(verbose)
			{
				// printf("\33[2KT\r");

				// cout << "Sensor " << s[i] << " 2sigma:";
			}

			for(int j = 0; j < cnst.n; j++)
			{
				if(verbose)
				{
					// printf("\r");
					if(j == 0)
					{
						// cout << "\t\t\tx = ";
					}
					else if(j == 1)
					{
						// cout << "\t\t\t\t\t\ty = ";
					}
					else if(j == 2)
					{
						// cout << "\t\t\t\t\t\t\t\t\tz = ";
					}
					
					// cout << 2.0*sqrt(e.P(j, j));
				}

				if(2.0*sqrt(e.P(j, j)) > stop_est_cov_thrsh)
				{
					loop = true;
				}
			}

			// cout << endl;
		}

		current_time = std::chrono::system_clock::now();
		delta_time = current_time - est_start;

		// cout << "Delta Time: " << delta_time.count() << endl;

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
		bool upd_params;
		param_str = "/bag_config_node/update_params";
		upd_params = false;
		if(CheckParam(param_str, 1, upd_params))
		{
			ros::param::get(param_str, upd_params);
		}

		if(upd_params)
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
				string warn_str = "Update to rosparam " + param_str + " has failed.";
				WARNING(warn_str);
			}
		}

		// cout << "DONE" << endl;
	}

	// Plot Calibration Results
	bool plot_c;
	param_str = "/bag_config_node/plot/calibration";
	plot_c = false;
	if(CheckParam(param_str, 1, plot_c))
	{
		ros::param::get(param_str, plot_c);
	}

	if(plot_c) // If the plot flag is set to true
	{
		for(int i = 0; i < s.size(); i++)
		{
			sensors[s[i]].plot_ekf(); // Plot the EKF results
			// sensors[s[i]].plot_e_y();
		}
	}
}


void SYSTEM::calibrate_pickup()
{
	// Selects the correct sensors to calibrate (Sensors 0 - 4)
	// 

	int cal_num_mark;

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
		cout << "Press Enter to calibrate." << endl;
		cin.get();

		param_str = "/bag_config_node/verbose";
		bool verbose = true;
		if(CheckParam(param_str, 1, verbose))
		{
			ros::param::get(param_str, verbose);
		}

		if(verbose)
		{
			cout << "Calibrating Pickup Sensors..." << endl;
		}

		param_str = "/bag_config_node/calibrator/pickup/num_markers";
		if(CheckParam(param_str, 2))
		{
			ros::param::get(param_str, cal_num_mark);
		}
		
		BAG calibrator_pickup(cal_num_mark);

		vector<double> m_i_pos;

		for(int i = 0; i < cal_num_mark; i++)
		{
			param_str = "/bag_config_node/calibrator/pickup/marker_";
			param_str = param_str + to_string(i);
			if(CheckParam(param_str, 2) && CheckParamSize(param_str, cnst.n))
			{
				ros::param::get(param_str, m_i_pos);
				calibrator_pickup.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
			}
		}

		assign_bag(calibrator_pickup);

		calibrate(s);
	}

	set_sensors_min(s); // Assign the minimal sensor object
}


void SYSTEM::calibrate_dropoff()
{
	// Selects the correct sensors to calibrate (Sensors 5 - 9)
	// 

	int cal_num_mark;

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
		cout << "Press Enter to calibrate." << endl;
		cin.get();

		param_str = "/bag_config_node/verbose";
		bool verbose = true;
		if(CheckParam(param_str, 1, verbose))
		{
			ros::param::get(param_str, verbose);
		}

		if(verbose)
		{
			cout << "Calibrating Dropoff Sensors..." << endl;
		}

		param_str = "/bag_config_node/calibrator/dropoff/num_markers";
		if(CheckParam(param_str, 2))
		{
			ros::param::get(param_str, cal_num_mark);
		}

		BAG calibrator_dropoff(cal_num_mark);

		vector<double> m_i_pos;

		for(int i = 0; i < cal_num_mark; i++)
		{
			param_str = "/bag_config_node/calibrator/dropoff/marker_";
			param_str = param_str + to_string(i);
			if(CheckParam(param_str, 2) && CheckParamSize(param_str, cnst.n))
			{
				ros::param::get(param_str, m_i_pos);
				calibrator_dropoff.markers[i].position = {m_i_pos[0], m_i_pos[1], m_i_pos[2]};
			}
		}

		assign_bag(calibrator_dropoff);

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
		EKF e = sensors[s[i]].ekf.back();

		cout << "Sensor " << i << " x = " << sensors[s[i]].position[0] << "\ty = " << sensors[s[i]].position[1] << "\tz = " << sensors[s[i]].position[2] << endl;
		cout << "\t2 Sigma:\tx = " << 2.0*sqrt(e.P(1,1)) << "\ty = " << 2.0*sqrt(e.P(2,2)) << "\tz = " << 2.0*sqrt(e.P(2,2)) << endl;

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

	// cout << "Package stamp" << package->stamp << "\t Previous msg time: " << prev_msg_time << endl;

	if(package->stamp != prev_msg_time)
	{
		// cout << "Package stamp " << package->stamp << "\tPrevious msg time: " << prev_msg_time << endl;

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

					Y.push_back(Y_k);
					bag.markers[cur_mid].estimate_pos(Y, sensors_min, core, cnst);

					Y.clear();
				}
			}

			Y_k.clear();
		}

		prev_msg_time = package->stamp;
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

	// ros::NodeHandle est_nh;

	// Create ros subscriber
	ros::Subscriber est_sub = est_nh.subscribe("arduino/data_read", 1000, &SYSTEM::estimator_callback, this);

	bool loop = true;
	EKF e;

	// Estimate continuously until an exit conditionrun_estimator is met
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

		// cout << "Time elapsed: " << time_elapsed.count() << endl;

		// If the time elapsed is greater than the set threshold stop looping
		if(time_elapsed.count() > stop_est_time)
		{
			loop = false;
			time_stop = true;
		}

		ros::spinOnce();
		// cout << "exited spin" << endl;
	}
	// cout << "Exited while loop" << endl;

	if(time_stop)
	{
		cout << "The necessary covariance threshold was not met before the maximum estimation time was reached." << endl;
	}
	else
	{
		cout << "Done" << endl;
	}

	// Plot the estimator restults
	bool plot_e;
	string param_str = "/bag_config_node/plot/estimation";
	plot_e = false;
	if(CheckParam(param_str, 1, plot_e))
	{
		ros::param::get(param_str, plot_e);
	}

	if(plot_e)
	{
		for(int i = 0; i < bag.markers.size(); i++)
		{
			if(bag.markers[i].updated)
			{
				bag.markers[i].plot_ekf();
				// bag.markers[i].plot_e_y();
			}
		}
	}

	cout << "Running Orientation Estimator..." << endl;

	int upd_markers = 0;

	for(int i = 0; i < bag.markers.size(); i++)
	{
		if(bag.markers[i].updated == true)
		{
			upd_markers++;
			cout << "M" << i << ":\t" << bag.markers[i].position[0] << "\t" << bag.markers[i].position[1] << "\t" << bag.markers[i].position[2] << endl;
		}
	}

	// cin.get();

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

	string param_str;

	param_str = "/bag_config_node/verbose";
	bool verbose = true;
	if(CheckParam(param_str, 1, verbose))
	{
		ros::param::get(param_str, verbose);
	}

	param_str = "/bag_config_node/run_estimation";
	bool run_est = true;
	if(CheckParam(param_str, 1, run_est))
	{
		ros::param::get(param_str, run_est);
	}

	if(run_est)
	{
		if(verbose)
		{
			cout << "Estimating pickup location." << endl;
		}

		string domain = "pickup";
		BAG bag_p;
		assign_bag(bag_p);

		bool mult_bags = false; // Assume there is a single bag in the core
		param_str = "/bag_config_node/multiple_bags";
		if(CheckParam(param_str, 1, mult_bags))
		{
			ros::param::get(param_str, mult_bags);
		}

		std::vector<int> s;

		for(int i = 0; i < 5; i++)
		{
			s.push_back(i);
		}

		run_estimator(s);

		if(!mult_bags)
		{
			upd_bag_config_msg(domain);
		}

		if(verbose && false)
		{
			cout << "Center: " << bag.center[0] << "  " << bag.center[1] << "  " << bag.center[2] << endl;
			cout << "Done estimating pickup location." << endl;
		}
	}
}


void SYSTEM::run_estimator_dropoff()
{
	// Function to run the estimation algorithm at the pickup location

	std::vector<int> s;

	string param_str;

	param_str = "/bag_config_node/verbose";
	bool verbose = true;
	if(CheckParam(param_str, 1, verbose))
	{
		ros::param::get(param_str, verbose);
	}

	param_str = "/bag_config_node/run_estimation";
	bool run_est = true;
	if(CheckParam(param_str, 1, run_est))
	{
		ros::param::get(param_str, run_est);
	}

	if(run_est)
	{
		if(verbose)
		{
			cout << "Estimating dropoff location." << endl;
		}

		string domain = "dropoff";
		BAG bag_d;
		assign_bag(bag_d);

		std::vector<int> s;

		for(int i = 5; i < 10; i++)
		{
			s.push_back(i);
		}

		run_estimator(s);

		upd_bag_config_msg(domain);

		if(verbose)
		{
			cout << "Center: " << bag.center[0] << "  " << bag.center[1] << "  " << bag.center[2] << endl;
			cout << "Done estimating dropoff location." << endl;
		}
	}
}


bool SYSTEM::observe_srv_callback(river_ros::Observe_srv::Request &req, river_ros::Observe_srv::Response &res)
{

	cout << "Srv Callback" << endl;

	res.observation_label = "cargo_not_found";

	res.time_received = ros::Time::now();

	reset_bag_config_msg();

	// if (we found all of the bags)
	if(false)
	{
		bag_config_msg.bags_found = true;
		bag_config_msg.observation_label = "cargo_found";
		bag_config_msg.pose_array.header.stamp = ros::Time::now();
		bag_config_msg.pose_array.poses.resize(1);
		bag_config_msg.domain_labels.resize(1);

		bag_config_msg.domain_labels[0] = "pickup location domain";
		bag_config_msg.pose_array.poses[0].position.x = -.67;
		bag_config_msg.pose_array.poses[0].position.y = 0.025;
		bag_config_msg.pose_array.poses[0].position.z = .37;
		bag_config_msg.pose_array.poses[0].orientation.x = .0024;
		bag_config_msg.pose_array.poses[0].orientation.y = -0.01842;
		bag_config_msg.pose_array.poses[0].orientation.z = .415616;
		bag_config_msg.pose_array.poses[0].orientation.w = .90934;

		/*
		bag_config_msg.domain_labels[1] = "pickup location domain";
		bag_config_msg.pose_array.poses[1].position.x = -.6;
		bag_config_msg.pose_array.poses[1].position.y = .6;
		bag_config_msg.pose_array.poses[1].position.z = .1;
		bag_config_msg.pose_array.poses[1].orientation.x = 0;
		bag_config_msg.pose_array.poses[1].orientation.y = 0;
		bag_config_msg.pose_array.poses[1].orientation.z = 1;
		bag_config_msg.pose_array.poses[1].orientation.w = 0;
		*/

		res.observation_label = "cargo_found";
	}
	else
	{

		run_estimator_pickup();

		if(bag.bag_found)
		{
			res.observation_label = "cargo_found";
		}
		else
		{
			res.observation_label = "cargo_not_found";
		}

		// run_estimator_dropoff();

		// if(bag.bag_found)
		// {
		// 	res.observation_label = "cargo_found";
		// }
		// else
		// {
		// 	res.observation_label = "cargo_not_found";
		// }
	}

	return true;
}


void SYSTEM::loop_estimator()
{
	string param_str = "/bag_config_node/run_estimation";
	bool run_est = true;
	if(CheckParam(param_str, 1, run_est))
	{
		ros::param::get(param_str, run_est);
	}

	if(run_est)
	{
		ros::ServiceServer service = est_nh.advertiseService("status/observe", &SYSTEM::observe_srv_callback, this);

		while(ros::ok())
		{
			send_bag_config_msg();

			ros::spinOnce();
		}
	}
}


void SYSTEM::reset_bag_config_msg()
{
	bag_config_msg.pose_array.poses.clear();
	bag_config_msg.domain_labels.clear();
	bag_config_msg.bags_found = false;
	bag_config_msg.observation_label = "cargo_not_found";
}


void SYSTEM::upd_bag_config_msg(string domain)
{

	// cout << "Updated message: ";

	bag_config_msg.pose_array.header.stamp = ros::Time::now();

	if(bag.bag_found)
	{
		// cout << "bag was found" << endl;
		bag_config_msg.bags_found = true;
		string domain_lbl;

		if(domain == "pickup")
		{
			domain_lbl = "pickup location domain";
		}
		else if(domain == "dropoff")
		{
			domain_lbl = "dropoff location domain";
		}
		bag_config_msg.domain_labels.push_back(domain_lbl);

		geometry_msgs::Pose tmp_pose;
		tmp_pose.position.x = bag.center[0];
		tmp_pose.position.y = bag.center[1];
		tmp_pose.position.z = bag.center[2];
		tmp_pose.orientation.x = bag.quat[0];
		tmp_pose.orientation.y = bag.quat[1];
		tmp_pose.orientation.z = bag.quat[2];
		tmp_pose.orientation.w = bag.quat[3];

		bag_config_msg.pose_array.poses.push_back(tmp_pose);
	}
}


void SYSTEM::send_bag_config_msg()
{
	ros::Publisher BagConfigPub = est_nh.advertise<river_ros::BagConfigPoseArray_msg>("bag_config/bag_configs", 10);

	// cout << "Bag config message sent: " << bag_config_msg.bags_found << endl;
	// cout << "Bags found = " << bag_config_msg.bags_found << endl;

	if(bag_config_msg.pose_array.poses.size() > 0)
	{
		// cout << "Position: " << bag_config_msg.pose_array.poses[0].position.x << "   " << bag_config_msg.pose_array.poses[0].position.y << "   " << bag_config_msg.pose_array.poses[0].position.z << endl;
	}

	BagConfigPub.publish(bag_config_msg);
	ros::spinOnce();
}


void SYSTEM::update_params()
{
	bool upd_params;
	string param_str = "/bag_config_node/update_params";
	upd_params = false;
	if(CheckParam(param_str, 1, upd_params))
	{
		ros::param::get(param_str, upd_params);
	}

	if(upd_params)
	{
		string param_path;
		string param_str = "/bag_config_node/pack_file_path";
		if(CheckParam(param_str, 1))
		{
			ros::param::get(param_str, param_path);
		
			stringstream cmd_strm;
			cmd_strm << "rosparam dump " << param_path << "river_ros/config/params.yaml /bag_config_node";
			string cmd_str = cmd_strm.str();
			const char *command = cmd_str.c_str();
			int garbage = system(command);
		}
	}
}


// vector<vector<vector<DATA>>> SYSTEM::reorg_data(vector<vector<vector<DATA>>> Y)
// {
// 	// Function to reorganize the data vector Y from Y[sid][k][mid] to Y[mid][k][sid]
// 	// 
// 	// Output:
// 	// 		Y_reorg - [vector<vector<vector<DATA>>>] Reorganized data vector
// 	// 

// 	vector<vector<vector<DATA>>> Y_reorg;
// 	vector<vector<DATA>> Y_k;
// 	vector<DATA> Y_sid;

// 	for(int mid = 0; mid < bag.markers.size(); mid++) // Loop through all possible marker IDs
// 	{
// 		for(int k = 0; k < Y[0].size(); k++) // Loop through all time steps k
// 		{
// 			for(int sid = 0; sid < Y.size(); sid++) // Loop through all sensors
// 			{
// 				if(Y[sid].size() > 0)
// 				{
// 					for(int m = 0; m < Y[sid][k].size(); m++) // Loop through all DATA objects
// 					{
// 						if(Y[sid][k][m].mid == mid) // If the measurement is for Marker mid, add it to the vector
// 						{
// 							Y_sid.push_back(Y[sid][k][m]);
// 							break;
// 						}
// 					}
// 				}
// 			}
// 			if(Y_sid.size() > 0) // Only add measurements if there are measurements to add
// 			{
// 				Y_k.push_back(Y_sid); // Add all measurements of mid at time k
// 				Y_sid.clear(); // Clear the vector for the next time k
// 			}
// 		}
// 		Y_reorg.push_back(Y_k); // Add all measurements of mid for all time k
// 		Y_k.clear(); // Clear the vector for the next marker ID
// 	}

// 	return Y_reorg;
// }
