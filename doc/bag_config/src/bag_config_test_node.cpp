#include "ros/ros.h"
#include "river_ros/Observe_srv.h"



bool observe_srvCB(river_ros::Observe_srv::Request& request, river_ros::Observe_srv::Response& response) {
	std::cout<<"im oBsErViNg look at me goooo"<<std::endl;
	response.time_received = ros::Time::now();
	response.observation_label = "cargo_found";
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "bag_config_test_node");
	ros::NodeHandle BC_test_NH;
	ros::ServiceServer observe_srv = BC_test_NH.advertiseService("status/observe_cargo",observe_srvCB);
	std::cout<<"ready BOI?"<<std::endl;
	ros::spin();
	return 0;
}
