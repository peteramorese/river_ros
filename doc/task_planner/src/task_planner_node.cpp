#include <cstdio>
#include <math.h>
#include <unordered_map>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "river_ros/PlanExecute_srv.h"
#include "river_ros/BagConfigPoseArray_msg.h"
#include "geometry_msgs/Pose.h"
#include "edge.h"
#include "astar.h"
#include "state.h"
#include "condition.h"
#include "transitionSystem.h"

class StartSrv {
	private:
		bool& begin;
	public: 
		StartSrv(bool& begin_) : begin(begin_) {
			begin = false;
		}
		bool startTP_serviceCB(river_ros::PlanExecute_srv::Request &request, river_ros::PlanExecute_srv::Response &response) {
			ROS_INFO_NAMED("task_planner_node", "Starting Task Planner");
			response.start_received = true;
			begin = true;
			return true;
		}
};

struct EnvironmentSub {
	int num_bags;
	bool bags_found;
	
	// This will hold the locations of the bags that are found, and will eventually hold
	// the hard-coded drop off locations (edited manually in the code)
	river_ros::BagConfigPoseArray_msg pose_array_copy; 

	// This maps the location labels to the array index for easy lookup
	std::unordered_map<std::string, int> label_ind_map;

	EnvironmentSub() {
		bags_found = false;
	}
	void envSubCB(const river_ros::BagConfigPoseArray_msg::ConstPtr& env_pose_array){
		std::cout<<"recieved msg BagConfigPoseArray"<<std::endl;
		bags_found = env_pose_array->bags_found;
		if (bags_found) {
			// Must copy over all data and store in local variable because msg is deallocated
			pose_array_copy.domain_labels = env_pose_array->domain_labels;
			pose_array_copy.pose_array.header.stamp = env_pose_array->pose_array.header.stamp;
			num_bags = env_pose_array->pose_array.poses.size();
			pose_array_copy.pose_array.poses.resize(num_bags);
			for (int i=0; i<num_bags; ++i)  {
				pose_array_copy.pose_array.poses[i].position.x = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].position.y = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].position.z = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].orientation.x = env_pose_array->pose_array.poses[i].orientation.x;
				pose_array_copy.pose_array.poses[i].orientation.y = env_pose_array->pose_array.poses[i].orientation.y;
				pose_array_copy.pose_array.poses[i].orientation.z = env_pose_array->pose_array.poses[i].orientation.z;
				pose_array_copy.pose_array.poses[i].orientation.w = env_pose_array->pose_array.poses[i].orientation.w;
				label_ind_map[pose_array_copy.domain_labels[i]] = i;
			}
		} else {
			std::cout<<" --bags were not found"<<std::endl;
		}
	}
	void mapSize(const std::string& label) {
		label_ind_map[label] = pose_array_copy.pose_array.poses.size() - 1;
	}
};

class DropOffLocations {
	private:
		struct coord {
			double x, y, z;
			std::string label;
		};
		std::vector<coord> locations;
		std::vector<bool> is_occupied;
		double max_r;
		// Store quaternion for all hard-coded drop off locations, this assumes
		// all bags should be placed in the same orientation
		double qx, qy, qz, qw;
	public:
		DropOffLocations(double max_r_) : max_r(max_r_) {}
		void addLocation(double x, double y, double z, const std::string& coord_label) {
			coord temp_coord;		
			temp_coord.x = x;
			temp_coord.y = y;
			temp_coord.z = z;
			temp_coord.label = coord_label;
			locations.push_back(temp_coord);
			is_occupied.push_back(false);
		}
		void getLocation(const std::string& coord_label, geometry_msgs::Pose& ret_pose) {
			for (int i=0; i<locations.size(); ++i) {
				if (locations[i].label == coord_label) {
					ret_pose.position.x = locations[i].x;
					ret_pose.position.y = locations[i].y;
					ret_pose.position.z = locations[i].z;
					ret_pose.orientation.x = qx;
					ret_pose.orientation.y = qy;
					ret_pose.orientation.z = qz;
					ret_pose.orientation.w = qw;
					break;
				}
			}
		}
		void getUnoccupiedLocation(geometry_msgs::Pose& ret_pose, std::string& ret_label) {
			// This will return the first unoccupied location, then set
			// that location to 'occupied'
			for (int i=0; i<locations.size(); ++i) {
				if (!is_occupied[i]) {
					ret_pose.position.x = locations[i].x;
					ret_pose.position.y = locations[i].y;
					ret_pose.position.z = locations[i].z;
					ret_pose.orientation.x = qx;
					ret_pose.orientation.y = qy;
					ret_pose.orientation.z = qz;
					ret_pose.orientation.w = qw;
					ret_label = locations[i].label;
					is_occupied[i] = true;
					break;
				}
			}
		}
		void setOrientation(double qx_, double qy_, double qz_, double qw_) {
			qx = qx_;	
			qy = qy_;	
			qz = qz_;	
			qw = qw_;	
		}
		double cartDist(double x, double y, double z, const coord& coord) {
			double sum = 0;
			sum += (x - coord.x)*(x - coord.x);
			sum += (y - coord.y)*(y - coord.y);
			sum += (z - coord.z)*(z - coord.z);
			sum = std::sqrt(sum);
			return sum;
		}
		int getNumLocs() {
			return locations.size();
		}
		bool getNearestLocLabel(double x, double y, double z, std::string& ret_coord_label) {
			// This will return the nearest location label, then set that location
			// to 'occupied', so that the same nearest location label cannot be 
			// returned twice
			double min_dist;
			int locations_ind;
			for (int i=0; i<locations.size(); ++i) {
				if (!is_occupied[i]) {
					double temp_dist;
					temp_dist = cartDist(x, y, z, locations[i]);
					if (i == 0 || temp_dist < min_dist) {
						min_dist = temp_dist;
						locations_ind = i;
					}
				}
			}
			if (min_dist < max_r) {
				ret_coord_label = locations[locations_ind].label;
				is_occupied[locations_ind] = true;
				return true;
			} else {
				return false;
				ROS_WARN("Did not find a location within the maximum radius");	
			}
		}
};

int main(int argc, char **argv){			
	ros::init(argc, argv, "task_planner_node");
	ros::NodeHandle TP_NH;

	// When client requests to plan and execute, begin once service is recieved
	bool begin;
	StartSrv start_srv_container(begin);
	ros::ServiceServer start_TP_service = TP_NH.advertiseService("status/plan_execute", &StartSrv::startTP_serviceCB, &start_srv_container);

	// Publish the status of the task planner
	ros::Publisher status_TP_pub = TP_NH.advertise<std_msgs::String>("task_planner/status", 10);
	std_msgs::String status_TP_msg;

	// Subscribe to the Bag Config topic to setup environmnet and call manipulator node
	EnvironmentSub env_sub_container;
	ros::Subscriber environment_TP_sub = TP_NH.subscribe("bag_config/bag_configs", 1, &EnvironmentSub::envSubCB, &env_sub_container);

	// Hard code drop off locations here
	// std::vector<std::vector<double>> drop_off_locs = {{-.4, .4, 0}, {-.4, -.4, 0}, {-.4, .4, .5}, {-.4, -.4, .5}};
	DropOffLocations drop_off_locs(1); // Maximum radius set in ctor
	drop_off_locs.setOrientation(0, 0, 0, 1);
	drop_off_locs.addLocation(-.4, .4, 0, "goal_L1");
	drop_off_locs.addLocation(-.4,-.4, 0, "goal_L2");
	drop_off_locs.addLocation(-.4, .4, .5, "goal_L3");
	drop_off_locs.addLocation(-.4,-.4, .5, "goal_L4");

	int hello = 0;
	ros::Rate r(1);
	while (ros::ok()) {
		r.sleep();
		hello++;
		std::cout<<"begin = "<<begin<<" hello = "<<hello<<std::endl;
		if (begin) {
			status_TP_msg.data = "working";
			status_TP_pub.publish(status_TP_msg);
			/* Set up the environment: */
			ros::spinOnce();

			ros::Rate r_env(1);
			while (!env_sub_container.bags_found && ros::ok()) {
				std::cout<<"Looking for pose array msg..."<<std::endl;
				ros::spinOnce();
				r_env.sleep();
			}

			if (env_sub_container.bags_found) {

				std::vector<std::string> var_labels_1 = {"safep", "safed"};
				std::vector<std::string> var_labels_2 = {"ee"};
				std::vector<std::string> var_labels_3 = {"true", "false"};
				std::vector<std::string> domain_pickup = {"safep"};
				std::vector<std::string> domain_loc_p;
				std::vector<std::string> domain_dropoff = {"safed"};
				std::vector<std::string> domain_loc_d;
				std::vector<std::string> domain_goal;
				
				// Keep track of how many bags are not found in the hard coded locs
				// so that goal locations can be added to match the number of objects
				int N_obj_elsewhere = 0; 
				int a; // Iterates thru all objects
				for (a=0; a<env_sub_container.pose_array_copy.pose_array.poses.size(); ++a) {
					std::string domain_label = env_sub_container.pose_array_copy.domain_labels[a];
					if (domain_label == "pickup location domain") {
						std::string label;
						{
							char label_buffer[100]; // sprintf buffer
							sprintf(label_buffer, "L_%d", a);
							label = label_buffer;
						}
						// Found bag, add this found bag as a location 
						// to the TP
						domain_pickup.push_back(label);
						domain_loc_p.push_back(label);
						var_labels_1.push_back(label);
						var_labels_2.push_back(label);
						N_obj_elsewhere++;
					} else if (domain_label == "dropoff location domain") {
						bool found;
						std::string label;
					        found = drop_off_locs.getNearestLocLabel(env_sub_container.pose_array_copy.pose_array.poses[a].position.x, env_sub_container.pose_array_copy.pose_array.poses[a].position.y, env_sub_container.pose_array_copy.pose_array.poses[a].position.z, label);
						if (found) {
							// Found bag, but it as already in a 
							// drop off location (hard-coded)
							geometry_msgs::Pose temp_pose;
							drop_off_locs.getLocation(label, temp_pose);
							env_sub_container.pose_array_copy.pose_array.poses.push_back(temp_pose);
							env_sub_container.pose_array_copy.domain_labels.push_back(label);
							env_sub_container.mapSize(label);
							domain_goal.push_back(label);
							domain_dropoff.push_back(label);
							var_labels_1.push_back(label);
							var_labels_2.push_back(label);
						} else {
							{
								char label_buffer[100]; // sprintf buffer
								sprintf(label_buffer, "L_%d", a);
								label = label_buffer;
							}
							// Found bag, add this found bag as a 
							// location to the TP
							domain_dropoff.push_back(label);
							domain_loc_d.push_back(label);
							var_labels_1.push_back(label);
							var_labels_2.push_back(label);
						}
					}
				}
				for (int i=0; i<N_obj_elsewhere; ++i) {
					std::string label;
					geometry_msgs::Pose temp_pose;
					drop_off_locs.getUnoccupiedLocation(temp_pose, label);
					env_sub_container.pose_array_copy.pose_array.poses.push_back(temp_pose);
					env_sub_container.pose_array_copy.domain_labels.push_back(label);
					env_sub_container.mapSize(label);
					domain_dropoff.push_back(label);
					domain_loc_d.push_back(label);
					var_labels_1.push_back(label);
					var_labels_2.push_back(label);
				}

				State::setStateDimension(var_labels_1, 0);
				for (int i=1; i<=a; ++i) {
					State::setStateDimension(var_labels_2, i);
				}
				State::setStateDimension(var_labels_3, a+1);
				State::setDomain("pickup domain", domain_pickup);
				State::setDomain("pickup location domain", domain_loc_p);
				State::setDomain("dropoff domain", domain_dropoff);
				State::setDomain("dropoff location domain", domain_loc_d);
				State::setDomain("goal domain", domain_goal);
				State::setStateDimensionLabel(0, "eeLoc");
				std::vector<bool> which_blocking = {false}; // first dim is eeLoc (not blocking)
				std::vector<std::string> obj_loc_group;
				std::vector<std::string> init_set_state = {"safep"};
				for (int i=1; i<=a; ++i) {
					std::string label;
					char label_buffer[100];
					sprintf(label_buffer, "obj%dLoc", i-1); // object labels are 0 index
					label = label_buffer;
					State::setStateDimensionLabel(i, label);
					obj_loc_group.push_back(label);
					which_blocking.push_back(true); // object locations are blocking
					init_set_state.push_back(var_labels_2[i]);
				}
				which_blocking.push_back(false);
				init_set_state.push_back("false");
				State::setStateDimensionLabel(a+1, "holding");
				State::setLabelGroup("object locations", obj_loc_group);
				BlockingState::setBlockingDim(which_blocking);

				BlockingState init_state;
				init_state.initNewSS();
				init_state.setState(init_set_state);




				/* ENVIRONMENT: SINGLE OBJECT, TWO DOMAINS (pickup dropoff), ONE LOCATION IN EACH */
				/*
				   std::vector<std::string> var_labels_1 = {"safep", "safed", "L1", "L2"};
				   std::vector<std::string> var_labels_2 = {"ee", "L1", "L2"};
				   std::vector<std::string> var_labels_3 = {"true", "false"};
				   std::vector<std::string> domain_pickup = {"L1","safep"};
				   std::vector<std::string> domain_loc_p = {"L1"};
				   std::vector<std::string> domain_dropoff = {"L2","safed"};
				   std::vector<std::string> domain_loc_d = {"L2"};

				   State::setStateDimension(var_labels_1, 0);
				   State::setStateDimension(var_labels_2, 1);
				   State::setStateDimension(var_labels_3, 2);
				   State::setDomain("pickup domain", domain_pickup);
				   State::setDomain("pickup location domain", domain_loc_p);
				   State::setDomain("dropoff domain", domain_dropoff);
				   State::setDomain("dropoff location domain", domain_loc_d);
				   State::setStateDimensionLabel(0, "eeLoc");
				   State::setStateDimensionLabel(1, "obj1Loc");
				   State::setStateDimensionLabel(2, "holding");
				   std::vector<std::string> grouperino = {"obj1Loc"};
				   State::setLabelGroup("object locations", grouperino);
				   std::vector<bool> which_blocking = {false, true, false};
				   BlockingState::setBlockingDim(which_blocking);

				   BlockingState init_state;
				   init_state.initNewSS();
				   std::vector<std::string> set_state_i = {"safep", "L1", "false"};
				   init_state.setState(set_state_i);
				   */

				/* ENVIRONMENT: TWO OBJECTS, TWO DOMAINS (pickup dropoff), TWO LOCATIONS IN PICKUP
				 * THREE LOCATIONS IN DROP OFF  */

				/*
				   std::vector<std::string> var_labels_1 = {"safep", "safed", "L1", "L2", "L3", "L4", "L5"};
				   std::vector<std::string> var_labels_2 = {"ee", "L1", "L2", "L3", "L4", "L5"};
				   std::vector<std::string> var_labels_3 = {"true", "false"};
				   std::vector<std::string> domain_pickup = {"L1", "L2", "safep"};
				   std::vector<std::string> domain_loc_p = {"L1", "L2"};
				   std::vector<std::string> domain_dropoff = {"L3", "L4", "L5", "safed"};
				   std::vector<std::string> domain_loc_d = {"L3", "L4", "L5"};

				   State::setStateDimension(var_labels_1, 0); // eef
				   State::setStateDimension(var_labels_2, 1); // obj1
				   State::setStateDimension(var_labels_2, 2); // obj2
				   State::setStateDimension(var_labels_3, 3);
				   State::setDomain("pickup domain", domain_pickup);
				   State::setDomain("pickup location domain", domain_loc_p);
				   State::setDomain("dropoff domain", domain_dropoff);
				   State::setDomain("dropoff location domain", domain_loc_d);
				   State::setStateDimensionLabel(0, "eeLoc");
				   State::setStateDimensionLabel(1, "obj1Loc");
				   State::setStateDimensionLabel(2, "obj2Loc");
				   State::setStateDimensionLabel(3, "holding");
				   std::vector<std::string> grouperino = {"obj1Loc", "obj2Loc"};
				   State::setLabelGroup("object locations", grouperino);
				   std::vector<bool> which_blocking = {false, true, true, false};
				   BlockingState::setBlockingDim(which_blocking);

				   BlockingState init_state;
				   init_state.initNewSS();
				   std::vector<std::string> set_state_i = {"safep", "L1", "L2", "false"};
				   init_state.setState(set_state_i);
				   */



				/* SET CONDITIONS */
				// Pickup domain conditions:
				std::vector<Condition> conds;
				std::vector<Condition*> cond_ptrs;
				conds.resize(10);
				cond_ptrs.resize(10);

				// Grasp in pickup domain
				conds[0].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup location domain");
				conds[0].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[0].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc",Condition::TRUE, "arg");
				conds[0].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[0].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee",Condition::TRUE, "arg");
				conds[0].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[0].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[0].setActionLabel("grasp");

				// Move around pickup domain with an object 
				conds[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup domain");
				conds[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
				conds[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
				conds[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable

				conds[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
				conds[1].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup domain");
				conds[1].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
				conds[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[1].setActionLabel("move");

				// Release the object that the eef is holding
				conds[2].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup location domain");
				conds[2].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
				conds[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::VAR, "ee",Condition::TRUE, "arg2");
				conds[2].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[2].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::TRUE, "arg2");
				conds[2].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[2].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[2].setActionLabel("release");

				// Move around pickup domain without an object
				conds[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup domain");
				conds[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
				conds[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
				conds[3].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "pickup domain");
				conds[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[3].setActionLabel("move");

				// Translate from pickup domain to drop off domain
				conds[4].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::EQUALS, Condition::VAR, "safep");
				conds[4].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[4].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::EQUALS, Condition::VAR, "safed");
				conds[4].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[4].setActionLabel("translate");

				// Dropoff domain conditions:

				// Grasp in dropoff domain
				conds[5].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff location domain");
				conds[5].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[5].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::TRUE, "arg");
				conds[5].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[5].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee", Condition::TRUE, "arg");
				conds[5].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[5].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[5].setActionLabel("grasp");

				// Move around dropoff domain with an object 
				conds[6].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[6].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff domain");
				conds[6].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
				conds[6].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
				conds[6].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable

				conds[6].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
				conds[6].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff domain");
				conds[6].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "na");
				conds[6].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[6].setActionLabel("move");

				// Release the object that the eef is holding
				conds[7].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff location domain");
				conds[7].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
				conds[7].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
				conds[7].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::VAR, "ee", Condition::TRUE, "arg2");
				conds[7].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[7].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::TRUE, "arg2");
				conds[7].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[7].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[7].setActionLabel("release");

				// Move around dropoff domain without an object
				conds[8].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
				conds[8].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff domain");
				conds[8].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
				conds[8].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[8].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg");
				conds[8].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "dropoff domain");
				conds[8].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[8].setActionLabel("move");

				// Translate from pickup domain to drop off domain
				conds[9].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::EQUALS, Condition::VAR, "safed");
				conds[9].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

				conds[9].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::EQUALS, Condition::VAR, "safep");
				conds[9].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
				conds[9].setActionLabel("translate");

				for (int i=0; i<conds.size(); ++i){
					cond_ptrs[i] = &conds[i];
					//conds[i].print();
				}


				/* Propositions */
				SimpleCondition p1;
				p1.addCondition(Condition::SIMPLE, Condition::GROUP, "object locations", Condition::IN_DOMAIN, Condition::DOMAIN, "goal domain");
				//p1.addCondition(Condition::SIMPLE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::LABEL, "Condition::NEGATE);
				p1.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
				p1.setLabel("p1");

				/* Transition System
				   Edge TS_graph(true);
				   TransitionSystem<BlockingState> TS(&TS_graph);
				   TS.setConditions(cond_ptrs);
				   TS.setInitState(&init_state);
				   TS.generate();
				//TS_graph.print();
				TS.print();
				*/

				/* DFA */
				Edge TS(true);
				Edge DFA(true);
				Edge PS(true);
				DFA.connect(0, 1, 0.4, "p1");
				DFA.connect(0, 0, 0.4, "!p1");
				//DFA.connect(1, 0, 0.4, "!p1");
				//DFA.connect(1, 1, 0.4, "p1");
				DFA.print();

				ProductSystem<BlockingState> prodsys(&TS, &DFA, &PS);
				// Transition system:
				prodsys.setConditions(cond_ptrs);
				prodsys.setInitState(&init_state);
				prodsys.generate();
				// Product system:
				prodsys.addProposition(&p1);
				prodsys.setAutomatonInitStateIndex(0);
				prodsys.addAutomatonAcceptingStateIndex(1);
				prodsys.compose();
				prodsys.print();
				std::vector<int> plan;
				float pathlength;
				prodsys.plan(plan);

				std::cout<<"\n";
				std::cout<<"Path: ";
				for (int i=0; i<plan.size(); ++i) {
					std::cout<<" -> "<<plan[i];	
				}
				std::cout<<"\n";

				/* Service Client */
				/*
				   ros::ServiceClient client1 = TP_NH.serviceClient<std_srvs::SetBool>("end_effector_status");
				//ros::ServiceClient client2 = TP_NH.serviceClient<river_ros::PlanExecute>("planex_service");
				std_srvs::SetBool eef_status;
				//river_ros::PlanExecute planex_service;
				eef_status.request.data = true;
				//planex_service.request.plan_and_execute = true;
				if (client1.call(eef_status)) {
				std::cout<<"i recieved: "<<eef_status.response.success<<std::endl;
				std::cout<<"with message: "<<eef_status.response.message<<std::endl;
				} else {
				ROS_ERROR("Failed to call service");
				return 1;
				}
				*/
				/*
				   if (client2.call(planex_service)) {
				   std::cout<<"i recieved: "<<planex_service.response.status<<std::endl;
				   } else {
				   ROS_ERROR("Failed to call service");
				   return 1;
				   }
				   */
				ros::WallDuration(5.0);
				status_TP_msg.data = "execute_success";
				status_TP_pub.publish(status_TP_msg);
				ros::spinOnce();
				std::cout<<"made it out phew"<<std::endl;
			}
		}
		begin = false;
		ros::spinOnce();
	}
	return 0;
}
