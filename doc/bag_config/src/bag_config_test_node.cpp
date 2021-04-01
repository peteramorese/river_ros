#include "ros/ros.h"
#include "river_ros/BagConfigPoseArray_msg.h"

/*
class BagConfig {
	private:
		bool bags_found;
		void observe() {
			
		}
	public:
		BagConfig() {
			bag_configs.poses.clear();
		}
		bool observe_srvCB(river_ros::Observe_srv::Request& request, river_ros::Observe_srv::Response& response) {
			std::cout<<"im oBsErViNg look at me goooo"<<std::endl;
			response.time_received = ros::Time::now();
			response.observation_label = "cargo_found";
			return true;
		}

};
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "bag_config_test_node");
	ros::NodeHandle BC_test_NH;

	//BagConfig observe_BC_container;
	//ros::ServiceServer observe_srv = BC_test_NH.advertiseService("status/observe_cargo",&BagConfig::observe_srvCB, &observe_BC_container);
	ros::Publisher observe_BC_pub = BC_test_NH.advertise<river_ros::BagConfigPoseArray_msg>("bag_config/bag_configs", 10);
	ros::Rate r(1);
	river_ros::BagConfigPoseArray_msg bag_configs;
	while (ros::ok()) {	
		// Put algorithm calls, arduino subscriber, etc here:

		// if (we found all of the bags)
		bag_configs.bags_found = true;
		bag_configs.observation_label = "cargo_found";
		// else
		// bag_configs.bags_found = false;
		// bag_configs.observation_label = "cargo_not_found";
		bag_configs.pose_array.header.stamp = ros::Time::now();
		bag_configs.pose_array.header.frame_id = "global_frame";
		bag_configs.pose_array.poses.resize(2);
		bag_configs.pose_array.poses[0].position.x = .4;
		bag_configs.pose_array.poses[0].position.y = .4;
		bag_configs.pose_array.poses[0].position.z = .4;
		bag_configs.pose_array.poses[0].orientation.x = 0;
		bag_configs.pose_array.poses[0].orientation.y = 0;
		bag_configs.pose_array.poses[0].orientation.z = 0;
		bag_configs.pose_array.poses[0].orientation.w = 1;

		// keep this 
		observe_BC_pub.publish(bag_configs);
		ros::spinOnce();
		r.sleep();
		std::cout<<"Published msg..."<<std::endl;
	}
	return 0;
}
