#include <cstdio>
#include <math.h>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/String.h"
#include "river_ros/PlanExecute_srv.h"
#include "river_ros/BagConfigPoseArray_msg.h"
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
	river_ros::BagConfigPoseArray_msg pose_array_copy;
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
			for (int i=0; i<num_bags; ++i)  {
				pose_array_copy.pose_array.poses[i].position.x = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].position.y = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].position.z = env_pose_array->pose_array.poses[i].position.x;
				pose_array_copy.pose_array.poses[i].orientation.x = env_pose_array->pose_array.poses[i].orientation.x;
				pose_array_copy.pose_array.poses[i].orientation.y = env_pose_array->pose_array.poses[i].orientation.y;
				pose_array_copy.pose_array.poses[i].orientation.z = env_pose_array->pose_array.poses[i].orientation.z;
				pose_array_copy.pose_array.poses[i].orientation.w = env_pose_array->pose_array.poses[i].orientation.w;
			}
		} else {
			std::cout<<" --bags were not found"<<std::endl;
		}
	}
};

class DropOffLocations {
	private:
		struct coord {
			double x, y, z;
			std::string label;
		};
		std::vector<coord> locations;
	public:
		void setLocCoords(double x, double y, double z, const std::string& coord_label) {
			coord temp_coord;		
			temp_coord.x = x;
			temp_coord.y = y;
			temp_coord.z = z;
			temp_coord.label = coord_label;
			locations.push_back(temp_coord);
		}
		double cartDist(const coord& coord1, const coord& coord2) {
			double sum = 0;
			sum += (coord1.x - coord2.x)*(coord1.x - coord2.x);
			sum += (coord1.y - coord2.y)*(coord1.y - coord2.y);
			sum += (coord1.z - coord2.z)*(coord1.z - coord2.z);
			sum = std::sqrt(sum);
			return sum;
		}
		void returnNearestLocLabel(double x, double y, double z, std::string& ret_coord_label) {
			float min_dist;
			for (int i=0; i<locations.size(); ++i) {
				
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

	int hello = 0;;
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
				
				// Keep track of how many bags are in the dropoff location
				// so that dropoff locations can be added to match the number of objects
				int N_obj = 0; 
				int a;
				for (a=0; a<env_sub_container.pose_array_copy.pose_array.poses.size(); ++a) {
					std::string domain_label = env_sub_container.pose_array_copy.domain_labels[a];
					if (domain_label == "pickup location domain") {
						std::string label;
						{
							char label_buffer[100]; // sprintf buffer
							sprintf(label_buffer, "L_%d", a);
							label = label_buffer;
						}
						// Found bag, add this found bag as a location to the TP
						domain_pickup.push_back(label);
						domain_loc_p.push_back(label);
						var_labels_1.push_back(label);
						var_labels_2.push_back(label);
					} else if (domain_label == "dropoff location domain") {
						std::string label;
						{
							char label_buffer[100]; // sprintf buffer
							sprintf(label_buffer, "L_%d", a);
							label = label_buffer;
						}
						// Found bag, add this found bag as a location to the TP
						domain_dropoff.push_back(label);
						domain_loc_d.push_back(label);
						var_labels_1.push_back(label);
						var_labels_2.push_back(label);
				
						
					}
					// For each found bag, make an empty location in the 
					// goal domain (dropoff domain)
					std::string label;
					{
						char label_buffer[100]; // sprintf buffer
						sprintf(label_buffer, "GoalL_%d", a);
						label = label_buffer;
					}	
					domain_dropoff.push_back(label);
					domain_goal.push_back(label);
					var_labels_1.push_back(label);
					var_labels_2.push_back(label);
					N_obj++;
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
