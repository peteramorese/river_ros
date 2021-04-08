#include <string>
#include "ros/ros.h"
#include "river_ros/BagConfigPoseArray_msg.h"
#include "river_ros/PlanExecute_srv.h"
#include "std_msgs/String.h"
#include "edge.h"

class ExecutionSub {
	private:
		std::string& ex_observation_label;
	public: 
		ExecutionSub(std::string& ex_observation_label_) : ex_observation_label(ex_observation_label_) {
			ex_observation_label = "working";			
		}
		void executionSubCB(const std_msgs::String::ConstPtr& ex_status) {
			ex_observation_label = ex_status->data;
		}
};

class ObserveSub {
	private: 
		bool& bags_found;	
		std::string obs_observation_label;
	public:
		ObserveSub(bool& bags_found_) : bags_found(bags_found_) {
			bags_found = false;
			obs_observation_label = "none";
		}
		void observeSubCB(const river_ros::BagConfigPoseArray_msg::ConstPtr& obs_bags_found) {
			std::cout<<"recieved msg: bags are found" <<std::endl;
			bags_found = obs_bags_found->bags_found;
			obs_observation_label = obs_bags_found->observation_label;
		}
		void getObservationLabel(std::string& observation_label) {
			observation_label = obs_observation_label;
		}
};
	


int main(int argc, char **argv) {
	ros::init(argc, argv, "status_node");
	ros::NodeHandle status_NH;

	// Create the super task graph 

	Edge super_tasks(true);
	super_tasks.connect(0, 1, 0.0f, "observe");
	super_tasks.connect(1, 0, 0.0f, "cargo_not_found");
	super_tasks.connect(1, 2, 0.0f, "cargo_found");
	super_tasks.connect(2, 3, 0.0f, "plan_execute");
	super_tasks.connect(3, 0, 0.0f, "execute_failure_observe");
	super_tasks.connect(3, 7, 0.0f, "execute_failure_fatal");
	super_tasks.connect(3, 4, 0.0f, "execute_success");
	super_tasks.connect(4, 5, 0.0f, "observe_check");
	super_tasks.connect(5, 6, 0.0f, "task_complete");
	super_tasks.connect(5, 0, 0.0f, "task_incomplete");
	super_tasks.connect(6, 8, 0.0f, "shutdown");
	super_tasks.connect(7, 8, 0.0f, "shutdown");

	int init_node = 0;
	int end_node = 8;
	int curr_node = init_node;
	bool action = true;
	std::string temp_action_label;
	std::string temp_observation_label;
	auto heads = super_tasks.getHeads();

	/* Services and Publishers/Subscribers */	

	// Retrieve bag configs
	//ros::ServiceClient observe_BC_sub = status_NH.subscribe("bag_config/bag_configs");
	//river_ros::Observe_srv observe_srv_msg;
	
	// Start task planner
	ros::ServiceClient plan_ex_client = status_NH.serviceClient<river_ros::PlanExecute_srv>("status/plan_execute");
	river_ros::PlanExecute_srv plan_ex_srv_msg;

	/* Super Task Planner */

	while (ros::ok() && curr_node != end_node) {
		auto currptr = heads[curr_node]->adjptr;
		if (action) {
			temp_action_label = currptr->label;
			ROS_INFO_NAMED("status_node","Current label: %s",temp_action_label.c_str());
			if (temp_action_label == "observe") {
				bool bags_found;
				ObserveSub obs_sub_container(bags_found);
				ros::Subscriber obs_sub = status_NH.subscribe("bag_config/bag_configs", 1, &ObserveSub::observeSubCB, &obs_sub_container);
				ros::Rate r_obs(1);
				while (ros::ok() && !bags_found) {
					std::cout<<"bags_found: "<<bags_found<<std::endl;
					ros::spinOnce();
					r_obs.sleep();
				}
				if (bags_found) {
					obs_sub_container.getObservationLabel(temp_observation_label);
				}
			} else if (temp_action_label == "plan_execute") {
				plan_ex_srv_msg.request.time_sent = ros::Time::now();
				if (plan_ex_client.call(plan_ex_srv_msg)) {
					if (plan_ex_srv_msg.response.start_received) {
						/* PUT ALL EXECUTION SAFETY CHECKS HERE */
						// Subscribe to task planner status while it executes
						ExecutionSub ex_sub_container(temp_observation_label);
						std::cout<<"Label being observed: " << temp_observation_label << std::endl;
						ros::Subscriber ex_sub = status_NH.subscribe("task_planner/status", 1, &ExecutionSub::executionSubCB, &ex_sub_container);
						ros::Rate r_ex(2); // Check twice every second
						while (ros::ok() && temp_observation_label == "working") {
							std::cout<<"checking ("<<temp_observation_label<<")..."<<std::endl;
							ros::spinOnce();	
							r_ex.sleep();
						}
					}	
				} else {
					ROS_ERROR_NAMED("status_node","Failed to call service: plan_execute");
				}
			} else if (temp_action_label == "observe_check") {
				// For now there is no check implemented
				/*
				observe_srv_msg.request.time_sent = ros::Time::now();
				if (observe_client.call(observe_srv_msg)) {
					temp_observation_label = observe_srv_msg.response.observation_label;	
					ROS_INFO_NAMED("status_node","Observation service response received. Observation time: "); // display observation time (to-do) 
				} else {
					ROS_ERROR_NAMED("status_node","Failed to call service: observe");
				}
				*/

				// This is copied from the "observe" action but this probably
				// should be some sort of service that checks the environment.
				// It does not need to be a service but there needs to be some sort
				// of verification that we are not using old data
				bool bags_found;
				ObserveSub obs_sub_container(bags_found);
				ros::Subscriber obs_sub = status_NH.subscribe("bag_config/bag_configs", 1, &ObserveSub::observeSubCB, &obs_sub_container);
				ros::Rate r_obs(1);
				while (ros::ok() && !bags_found) {
					ros::spinOnce();
					r_obs.sleep();
				}
				if (bags_found) {
					//getObservationLabel(temp_observation_label);
					temp_observation_label = "task_complete";
				}
			} else if (temp_action_label == "shutdown") {
				ROS_INFO_NAMED("status_node","Shutting down...");
			} else {
				ROS_ERROR_NAMED("status_node","Unrecognized super action");
			}
			action = false;
			curr_node = currptr->nodeind;
		} else {
			bool observation_found = false;
			//ros::Rate r(1);
			while (ros::ok() && currptr != nullptr) {
				//r.sleep();
				if (currptr->label == temp_observation_label) {
					curr_node = currptr->nodeind;
					observation_found = true;
					break;
				}
				currptr = currptr->adjptr;
			}
			if (observation_found) {
				action = true;
				ROS_INFO_NAMED("status_node","Observed: %s", temp_observation_label.c_str());		
			} else {
				ROS_ERROR_NAMED("status_node","Did not find valid observation. Did a service fail?");
			}
		}
	
	}

	ros::spin();
	return 0;
}
